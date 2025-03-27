#include "RISCV.h"
#include "RISCVSubtarget.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

namespace {

class RISCVLIWBundling : public MachineFunctionPass {
public:
    static char ID;
    RISCVLIWBundling(size_t _maxBundleSize, bool _fixed) : MachineFunctionPass(ID), maxBundleSize(_maxBundleSize), fixed(_fixed) {
        if (fixed) {
            switch (maxBundleSize) {
            case 2: {
                slots.push_back({
                        RISCV::OPCLOAD, RISCV::OPCSTORE, RISCV::OPCMISCMEM, RISCV::OPCOPDIV,
                        RISCV::OPCOPMUL, RISCV::OPCSYSTEM, RISCV::OPCOP32DIV, RISCV::OPCOP32MUL,
                        RISCV::OPCAUIPCJALR});
                slots.push_back({RISCV::OPCBRANCH, RISCV::OPCJALR, RISCV::OPCJAL});
                break;
            }
            case 4: {
                slots.push_back({RISCV::OPCSYSTEM, RISCV::OPCMISCMEM,
			         RISCV::OPCOPDIV, RISCV::OPCOP32DIV});
                slots.push_back({RISCV::OPCOPMUL, RISCV::OPCOP32MUL});
                slots.push_back({RISCV::OPCAUIPCJALR, RISCV::OPCLOAD, RISCV::OPCSTORE});
                slots.push_back({RISCV::OPCBRANCH, RISCV::OPCJALR, RISCV::OPCJAL});
                break;
            }
            default: {
                assert(false);
            }
            }
            // Any-slot instructions
            for (std::set<RISCV::RVOPC> &slot : slots) {
                slot.insert(RISCV::OPCOPIMM);
                slot.insert(RISCV::OPCOP);
                slot.insert(RISCV::OPCAUIPC);
                slot.insert(RISCV::OPCLUI);
                slot.insert(RISCV::OPCOPIMM32);
                slot.insert(RISCV::OPCOP32);
            }
        } else {
            // First slot is special
            // The AUIPC+JALR pair will get expanded by the linker into two insns
            slots.push_back({RISCV::OPCLOAD, RISCV::OPCOPIMM, RISCV::OPCOP, RISCV::OPCAUIPCJALR});
            // other slots can be anything
            for (size_t i = 1; i < maxBundleSize; i++) {
                slots.push_back({RISCV::OPCLOAD, RISCV::OPCOPIMM, RISCV::OPCOP,
                        RISCV::OPCSTORE, RISCV::OPCBRANCH, RISCV::OPCJALR, RISCV::OPCMISCMEM,
                        RISCV::OPCJAL, RISCV::OPCOPDIV, RISCV::OPCOPMUL, RISCV::OPCSYSTEM,
                        RISCV::OPCAUIPC, RISCV::OPCLUI,
                        RISCV::OPCOPIMM32, RISCV::OPCOP32, RISCV::OPCOP32DIV, RISCV::OPCOP32MUL
                    });
            }
        }
    }

    size_t maxBundleSize;
    bool fixed;

    std::vector<std::set<RISCV::RVOPC>> slots;
    std::vector<MachineInstr*> currentBundle;
    std::vector<std::vector<MachineInstr*>> blockBundles;

    bool addToCurrentBundle(MachineInstr *MI) {
        // InlineASM always needs to be its own bundle or set of bundles
        if (MI->getOpcode() == TargetOpcode::INLINEASM) {
            for (MachineInstr* MI : currentBundle) if (MI) return false;

            // Must go to slot0
            currentBundle[0] = MI;
            return true;
        }
        for (MachineInstr* MI : currentBundle) if (MI && MI->getOpcode() == TargetOpcode::INLINEASM) return false;

        RISCV::RVOPC opcode = RISCV::getRVOpcode(MI);

        // Check for hazards
        // Pseudo CALL/TAIL operands
	size_t earliestSlot = 0; // for WARs, if a WAR, don't reorder the younger instruction earlier than here to maintain spike compat
        for (const auto &O : MI->operands()) {
            for (size_t i = 0; i < maxBundleSize; i++) {
		MachineInstr *PMI = currentBundle[i];
                if (PMI) {
                    for (MachineOperand &PO : PMI->operands()) {
			if (O.isReg() && !O.isImplicit() && PO.isReg() && O.getReg() == PO.getReg()) {
			    // RAWs
			    if (PO.isDef() && !O.isDef()) { return false; }
			    // WARs (for spike compatibility)
			    if (!PO.isDef() && O.isDef()) { earliestSlot = i; }
			}
                    }
                }
            }
        }
	// X1 is a read operand of PseudoRET
	if (MI->getOpcode() == RISCV::PseudoRET) {
	    for (MachineInstr *PMI : currentBundle) {
                if (PMI) {
                    for (MachineOperand &PO : PMI->operands()) {
                        if (PO.isReg() && PO.isDef() && RISCV::X1 == PO.getReg()) {
                            return false;
                        }
                    }
                }
            }
	}

        // Special handling for PseudoCALL/TAIL, which gets expanded by the linker
        if (opcode == RISCV::OPCAUIPCJALR) {
            if (fixed) {
                // PseudoCALL/TAIL gets expanded to AUIPC+JALR, so both slots must be clear
                if (currentBundle[maxBundleSize-2] != nullptr || currentBundle[maxBundleSize-1] != nullptr) {
                    return false;
                }
            } else {
                // PseudoCALL/TAIL goes only in slot0, other slots must be clear
                for (MachineInstr* I : currentBundle) if (I != nullptr) return false;
            }
        }

        for (size_t i = earliestSlot; i < maxBundleSize; i++) {
            if (currentBundle[i] == nullptr && slots[i].find(opcode) != slots[i].end()) {
                currentBundle[i] = MI;
                return true;
            }
        }
        return false;
    }

    MachineInstr* generateNop(MachineFunction &MF) {
        const TargetInstrInfo *TII = MF.getSubtarget().getInstrInfo();

        MachineInstr* MI = BuildMI(MF, DebugLoc(), TII->get(RISCV::ADDI))
                        .addReg(RISCV::X0, RegState::Define)  // Destination register
                        .addReg(RISCV::X0, RegState::Kill)    // Source register (X0)
                        .addImm(0);
        return MI;
    }

    void legalizeAndEmitCurrentBundle(MachineFunction &MF) {
        size_t bundleInsns = 0;
        for (MachineInstr* MI : currentBundle) if (MI) bundleInsns++;
        if (bundleInsns == 0) return;

        if (!fixed) {
            if (bundleInsns == 1) {
                // Move the single instruction to the header slot
                for (size_t i = 1; i < maxBundleSize; i++) {
                    if (currentBundle[i]) {
                        currentBundle[0] = currentBundle[i];
                        currentBundle[i] = nullptr;
                    }
                }
            } else if (bundleInsns > 1 && currentBundle[0] == nullptr) {
                // Inject a nop bundle-header
                currentBundle[0] = generateNop(MF);
            }
        } else {
            if (!(currentBundle[0] && currentBundle[0]->getOpcode() == TargetOpcode::INLINEASM)) {
                for (size_t i = 0; i < maxBundleSize; i++) {
                    // Linker relaxation will fill 2 slots, don't generate the second nop
                    if (currentBundle[i] && RISCV::getRVOpcode(currentBundle[i]) == RISCV::OPCAUIPCJALR) break;
                    if (!currentBundle[i]) currentBundle[i] = generateNop(MF);
                }
            }
        }
        blockBundles.push_back(currentBundle);
        currentBundle = std::vector<MachineInstr*>(maxBundleSize, nullptr);
    }

    bool runOnMachineBasicBlock(MachineBasicBlock &MBB, const TargetInstrInfo* TII) {
        MachineFunction* MF = MBB.getParent();

        // align the start of a bundle
        // To work around odd linker-relaxation behavior (possibly buggy), we manually
        // inject no-ops in assembly emission
        if (fixed)
            MBB.setAlignment(Align(maxBundleSize * 4));

        for (auto I = MBB.begin(), E = MBB.end(); I != E; ) {
            MachineInstr &MI = *I++;
            // Remove CFI_INSTRUCTIONs (these are just used for generating debug info)
            if (MI.isCFIInstruction() || MI.isKill() || MI.getOpcode() == TargetOpcode::IMPLICIT_DEF) {
                MBB.erase(&MI);
            }
        }

        blockBundles.clear();
        currentBundle = std::vector<MachineInstr*>(maxBundleSize, nullptr);

        // Create sequence of bundles
        for (MachineInstr &MI : MBB) {
            if (!addToCurrentBundle(&MI)) {
                legalizeAndEmitCurrentBundle(*MF);
                assert(addToCurrentBundle(&MI));
            }

            // No younger instructions can be placed in the bundle with a PseudoCALL/TAIL
            if (RISCV::getRVOpcode(&MI) == RISCV::OPCAUIPCJALR)
                legalizeAndEmitCurrentBundle(*MF);
        }
        legalizeAndEmitCurrentBundle(*MF);

        // Clear the basic block
	for (auto I = MBB.begin(), E = MBB.end(); I != E; ) {
            MachineInstr &MI = *I++;
	    MBB.remove(&MI);
	}

        // Add bundled instructions to basicblock
        for (std::vector<MachineInstr*> &bundle : blockBundles) {
            for (size_t i = 0; i < maxBundleSize; i++) {
                if (bundle[i]) {
                    MBB.push_back(bundle[i]);
                    if (i > 0) bundle[i]->bundleWithPred();
                }
            }
        }
        return false;
    }

    bool runOnMachineFunction(MachineFunction &MF) override {
        const TargetInstrInfo *TII = MF.getSubtarget().getInstrInfo();

        if (maxBundleSize > 1) {
          for (MachineBasicBlock &MBB : MF) runOnMachineBasicBlock(MBB, TII);
        }
        MF.setAlignment(Align(16));
        return false; // Return true if the function was modified
    }
};
} // end anonymous namespace

char RISCVLIWBundling::ID = 0;

INITIALIZE_PASS(RISCVLIWBundling, "riscv-vliw-bundle", "RISC-V VLIW Bundling", false, false)

FunctionPass *llvm::createRISCVLIWBundlingPass(size_t maxBundleSize, bool fixed) {
  return new RISCVLIWBundling(maxBundleSize, fixed);
}
