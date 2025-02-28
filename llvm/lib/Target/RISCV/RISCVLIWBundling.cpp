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
    RISCVLIWBundling(size_t maxBundleSize) : MachineFunctionPass(ID), maxBundleSize(maxBundleSize) {}

    size_t maxBundleSize;
    std::vector<MachineInstr*> currentBundle;
    std::vector<MachineOperand*> currentBundleWrites;

    void swapInsns(MachineInstr *A, MachineInstr *B) {
        MachineBasicBlock *MBB = A->getParent();
        auto NextB = B->getIterator();
        NextB++;
        MBB->remove(B);
        MBB->insert(A->getIterator(), B);
        MBB->remove(A);
        MBB->insert(NextB, A);
    }

    bool legalHead(const MachineInstr* MI) {
        switch (MI->getOpcode()) {
        case RISCV::LB:
        case RISCV::LBU:
        case RISCV::LH:
        case RISCV::LHU:
        case RISCV::LW:
        case RISCV::LWU:
        case RISCV::LD:

        case RISCV::ADDI:
        case RISCV::SLTI:
        case RISCV::SLTIU:
        case RISCV::XORI:
        case RISCV::ORI:
        case RISCV::ANDI:
        case RISCV::SLLI:
        case RISCV::SRLI:
        case RISCV::SRAI:

        case RISCV::ADD:
        case RISCV::SUB:
        case RISCV::SLL:
        case RISCV::SLT:
        case RISCV::SLTU:
        case RISCV::XOR:
        case RISCV::SRL:
        case RISCV::SRA:
        case RISCV::OR:
        case RISCV::AND:
        case RISCV::MUL:
        case RISCV::MULH:
        case RISCV::MULHSU:
        case RISCV::MULHU:
        case RISCV::DIV:
        case RISCV::DIVU:
        case RISCV::REM:
        case RISCV::REMU:
            return true;
        default:
            return false;
        }
    }

    void emitBundle() {
        currentBundleWrites.clear();
        if (currentBundle.size() == 1) {
            errs() << "  Inst: " << *currentBundle[0];
        } else if (currentBundle.size() > 1) {
            errs() << "  Detected bundle:\n";

            bool legal = legalHead(currentBundle[0]);

            if (!legal) {
                // try to swap head with a legal head
                for (size_t i = 1; i < currentBundle.size(); i++) {
                    if (legalHead(currentBundle[i])) {
                        swapInsns(currentBundle[0], currentBundle[i]);
                        MachineInstr* t = currentBundle[0];
                        currentBundle[0] = currentBundle[i];
                        currentBundle[i] = t;
                        legal = true;
                        break;
                    }
                }
                // no valid instruction in bundle
                if (!legal) {
                    currentBundle.clear();
                    return;
                }
            }

            errs() << "    Head : " << *currentBundle[0];
            for (size_t i = 1; i < currentBundle.size(); i++) {
                errs() << "    Tail : " << *currentBundle[i];
                currentBundle[i]->bundleWithPred();
            }
        }
        currentBundle.clear();
    }

    void addToCurrentBundle(MachineInstr* I) {
        currentBundle.push_back(I);
        for (MachineOperand &O : I->operands()) {
            if (O.isReg() && O.isDef())
                currentBundleWrites.push_back(&O);
        }
    }

    bool runOnMachineBasicBlock(MachineBasicBlock &BB, const TargetInstrInfo* TII) {
        // Process each instruction in the basic block
        errs() << "BasicBlock: " << BB.getName() << "\n";

        for (MachineInstr &MI : BB) {
            MCInstrDesc D = MI.getDesc();
            if (D.isBranch() || D.isCall()) {
                addToCurrentBundle(&MI);
                emitBundle();
            } else {
                bool hasRAW = false;
		unsigned opcode = MI.getOpcode();
		// bool mustBeInOwnBundle = opcode == RISCV::JAL || opcode == RISCV::JALR || opcode == RISCV::PseudoCALL;
		for (const auto &O : MI.operands()) {
                    if (O.isReg()) {
                        for (MachineOperand *PO : currentBundleWrites) {
                            if (O.getReg() == PO->getReg()) {
                                hasRAW = true;
                            }
                        }
                    }

                }

                if (hasRAW) emitBundle();
                addToCurrentBundle(&MI);
                if (currentBundle.size() >= maxBundleSize ) emitBundle();
            }
        }
        // Handle the last remaining bundle if any
        emitBundle();

        for (MachineInstr &MI : BB.instrs()) {
            errs() << "After bundling: " << MI;
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

FunctionPass *llvm::createRISCVLIWBundlingPass(size_t maxBundleSize) {
  return new RISCVLIWBundling(maxBundleSize);
}
