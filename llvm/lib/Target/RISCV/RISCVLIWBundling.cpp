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
    RISCVLIWBundling(unsigned v) : MachineFunctionPass(ID), variant(v) {
        switch (variant) {
        case RISCV::FeatureStdExtXRVLIWFQ: {
            maxBundleSize = 4;
            slots.push_back({RISCV::OPCSYSTEM, RISCV::OPCMISCMEM, RISCV::OPCLOAD, RISCV::OPCSTORE,
                    RISCV::OPCOPDIV, RISCV::OPCOP32DIV});
            slots.push_back({RISCV::OPCOPMUL, RISCV::OPCOP32MUL});
            slots.push_back({RISCV::OPCAUIPCJALR});
            slots.push_back({RISCV::OPCBRANCH, RISCV::OPCJALR, RISCV::OPCJAL});
            // any-slot insns
            for (std::set<RISCV::RVOPC> &slot : slots) {
                slot.insert(RISCV::OPCOPIMM);
                slot.insert(RISCV::OPCOP);
                slot.insert(RISCV::OPCAUIPC);
                slot.insert(RISCV::OPCLUI);
                slot.insert(RISCV::OPCOPIMM32);
                slot.insert(RISCV::OPCOP32);
            }
            // don't need to initialize counts, the slots are fixed
            break;
        }
        case RISCV::FeatureStdExtXRVLIWSQ: {
            maxBundleSize = 4;
            // Only restrict AUIPCJALR to 2nd to last slot, and BR/JMP to last slot,
            // otherwise any insn can go anywhere
            for (size_t i = 0; i < maxBundleSize; i++) {
                slots.push_back({RISCV::OPCSYSTEM, RISCV::OPCMISCMEM, RISCV::OPCLOAD, RISCV::OPCSTORE,
                        RISCV::OPCOPDIV, RISCV::OPCOP32DIV, RISCV::OPCOPMUL, RISCV::OPCOP32MUL,
                        RISCV::OPCOPIMM, RISCV::OPCOP, RISCV::OPCAUIPC, RISCV::OPCLUI,
                        RISCV::OPCOPIMM32, RISCV::OPCOP32});
            }
            slots[2].insert(RISCV::OPCAUIPCJALR);
            slots[3].insert(RISCV::OPCBRANCH);
            slots[3].insert(RISCV::OPCJAL);
            slots[3].insert(RISCV::OPCJALR);

            counts.push_back(std::make_pair(std::set<RISCV::RVOPC>({RISCV::OPCSYSTEM, RISCV::OPCMISCMEM,
                            RISCV::OPCLOAD, RISCV::OPCSTORE,
                            RISCV::OPCOPDIV, RISCV::OPCOP32DIV
                        }), 1));
            counts.push_back(std::make_pair(std::set<RISCV::RVOPC>({RISCV::OPCOPMUL, RISCV::OPCOP32MUL}), 1));
            break;
        }
	case RISCV::FeatureStdExtXRVLIWHQ: {
	    maxBundleSize = 4;
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

	    counts.push_back(std::make_pair(std::set<RISCV::RVOPC>({RISCV::OPCSYSTEM, RISCV::OPCMISCMEM,
                            RISCV::OPCLOAD, RISCV::OPCSTORE,
                            RISCV::OPCOPDIV, RISCV::OPCOP32DIV
                        }), 1));
            counts.push_back(std::make_pair(std::set<RISCV::RVOPC>({RISCV::OPCOPMUL, RISCV::OPCOP32MUL}), 1));
	    break;
	}
        default: {
            assert(false);
        }
        }

    }

    unsigned variant;
    size_t maxBundleSize;
    std::vector<std::set<RISCV::RVOPC>> slots;
    std::vector<std::pair<std::set<RISCV::RVOPC>,size_t>> counts;
    std::vector<size_t> countsSoFar;
    std::vector<MachineInstr*> currentBundle;
    std::vector<std::vector<MachineInstr*>> blockBundles;

    bool addToCurrentBundle(MachineInstr *MI) {
        // InlineASM always needs to be its own bundle
        if (MI->getOpcode() == TargetOpcode::INLINEASM) {
            for (MachineInstr* MI : currentBundle) if (MI) return false;

            // Must go to slot0
            currentBundle[0] = MI;
            return true;
        }

        RISCV::RVOPC opcode = RISCV::getRVOpcode(MI);
	// fences must be in their own bundle
	if (opcode == RISCV::RVOPC::OPCMISCMEM) {
	  for (MachineInstr* MI : currentBundle) if (MI) return false;
	}

	for (MachineInstr* MI : currentBundle) {
	  if (MI) {
	    RISCV::RVOPC po = RISCV::getRVOpcode(MI);
	    if (MI->getOpcode() == TargetOpcode::INLINEASM ||
		po == RISCV::RVOPC::OPCMISCMEM ||
		po == RISCV::RVOPC::OPCBRANCH ||
		po == RISCV::RVOPC::OPCJALR ||
		po == RISCV::RVOPC::OPCAUIPCJALR)
	      return false;
	  }
	}

        // Check for hazards
        // Pseudo CALL/TAIL operands
	size_t earliestSlot = 0; // for WARs, if a WAR, don't reorder the younger instruction earlier than here to maintain spike compat
        std::vector<Register> reads;
        std::vector<Register> writes;
        for (const auto &O : MI->operands()) {
            if (O.isReg() && !O.isImplicit()) {
                if (O.isDef()) {
                    writes.push_back(O.getReg());
                } else {
                    reads.push_back(O.getReg());
                }
            }
            // This variant changes the emissiom for the bundle header to drop
            // the tprel/pcrel hi/lo symbols, so any instruction which has a tprel/pcrel hi/lo
            // operand cannot be the header
            if (variant == RISCV::FeatureStdExtXRVLIWHQ) {
                switch (O.getTargetFlags()) {
		case RISCVII::MO_TPREL_LO:
		case RISCVII::MO_TPREL_HI:
		case RISCVII::MO_TPREL_ADD:
                case RISCVII::MO_PCREL_HI:
                case RISCVII::MO_PCREL_LO:
                    earliestSlot = 1;
                    break;
                default:
                    break;
                }
            }
        }
        // X1 is a read operand of PseudoRET
        if (MI->getOpcode() == RISCV::PseudoRET) {
            reads.push_back(RISCV::X1);
        }

        // Check RAWs and WARs
        for (size_t i = 0; i < maxBundleSize; i++) {
            MachineInstr *PMI = currentBundle[i];
            if (PMI) {
                for (MachineOperand &PO : PMI->operands()) {
                    if (PO.isReg()) {
                        if (PO.isDef()) { // RAWs
                            for (Register r : reads) {
                                if (PO.getReg() == r) { return false; }
                            }
                        } else { // WARs (for spike compatibility)
                            for (Register w : writes) {
                                if (PO.getReg() == w) { earliestSlot = i; }
                            }
                        }
                    }
                }
            }
        }

        // Special handling for PseudoCALL/TAIL, which gets expanded by the linker
        if (opcode == RISCV::OPCAUIPCJALR) {
            if (variant == RISCV::FeatureStdExtXRVLIWFQ || variant == RISCV::FeatureStdExtXRVLIWSQ) {
                // PseudoCALL/TAIL gets expanded to AUIPC+JALR, so both slots must be clear
                if (currentBundle[maxBundleSize-2] != nullptr || currentBundle[maxBundleSize-1] != nullptr) {
                    return false;
                }
            } else {
                // PseudoCALL/TAIL goes only in slot0, other slots must be clear
		for (MachineInstr* I : currentBundle) if (I) return false;
            }
        }

        // Check the opcode count in this bundle hasn't been violated
        for (size_t i = 0; i < counts.size(); i++) {
            if (counts[i].first.count(opcode) > 0) {
                // Already have too many instructions of this category
                if (countsSoFar[i] == counts[i].second) {
                    return false;
                }
            }
        }

        // Check the instruction can find a slot
        for (size_t i = earliestSlot; i < maxBundleSize; i++) {
            if (currentBundle[i] == nullptr && slots[i].find(opcode) != slots[i].end()) {
                for (size_t j = 0; j < counts.size(); j++) {
                    if (counts[j].first.count(opcode) > 0) {
                        countsSoFar[j]++;
                    }
                }
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

        if (variant == RISCV::FeatureStdExtXRVLIWHQ) {
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
        countsSoFar = std::vector<size_t>(counts.size(), 0);
    }

    bool runOnMachineBasicBlock(MachineBasicBlock &MBB, const TargetInstrInfo* TII) {
        MachineFunction* MF = MBB.getParent();

        // align the start of a bundle
        // To work around odd linker-relaxation behavior (possibly buggy), we manually
        // inject no-ops in assembly emission
        if ((variant == RISCV::FeatureStdExtXRVLIWFQ || variant == RISCV::FeatureStdExtXRVLIWSQ) &&
            !MBB.getParent()->getSubtarget<RISCVSubtarget>().hasStdExtCOrZca()) {
            MBB.setAlignment(Align(maxBundleSize * 4));
        }

        for (auto I = MBB.begin(), E = MBB.end(); I != E; ) {
            MachineInstr &MI = *I++;
            // Remove CFI_INSTRUCTIONs (these are just used for generating debug info)
            if (MI.isCFIInstruction() || MI.isKill() || MI.getOpcode() == TargetOpcode::IMPLICIT_DEF) {
                MBB.erase(&MI);
            }
        }

        blockBundles.clear();
        currentBundle = std::vector<MachineInstr*>(maxBundleSize, nullptr);
        countsSoFar = std::vector<size_t>(counts.size(), 0);

        // Create sequence of bundles
        for (MachineInstr &MI : MBB) {
            if (!addToCurrentBundle(&MI)) {
                legalizeAndEmitCurrentBundle(*MF);
                assert(addToCurrentBundle(&MI));
            }

            // No younger instructions can be placed in the bundle with a PseudoCALL/TAIL or a INLINEASM
            if (MI.getOpcode() == TargetOpcode::INLINEASM || RISCV::getRVOpcode(&MI) == RISCV::OPCAUIPCJALR)
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

FunctionPass *llvm::createRISCVLIWBundlingPass(unsigned variant) {
  return new RISCVLIWBundling(variant);
}
