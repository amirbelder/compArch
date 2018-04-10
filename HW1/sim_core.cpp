/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

SIM_coreState state;

#define STAGE_IF    0
#define STAGE_ID    1
#define STAGE_EX    2
#define STAGE_MEM   3
#define STAGE_WB    4

void reset_cmd(SIM_cmd* cmd) {
  memset(cmd, 0, sizeof(SIM_cmd));
}

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
int SIM_CoreReset(void) {
  state.pc = 0;
  for (int i = 0; i <= SIM_REGFILE_SIZE; i ++)
    state.regFile[i] = 0;
  memset(&state, 0, sizeof(SIM_coreState));
  SIM_MemInstRead(state.pc, &state.pipeStageState[STAGE_IF].cmd);
  return 0;
}

void DoIDStage() {
    state.pipeStageState[STAGE_ID].src1Val = state.regFile[state.pipeStageState[STAGE_ID].cmd.src1];
    if (state.pipeStageState[STAGE_ID].cmd.isSrc2Imm) {
      state.pipeStageState[STAGE_ID].src2Val = state.pipeStageState[STAGE_ID].cmd.src2;
    }
    else {
      state.pipeStageState[STAGE_ID].src2Val = state.regFile[state.pipeStageState[STAGE_ID].cmd.src2];
    }
}

void DoEXStage() {
  PipeStageState* current_stage = &state.pipeStageState[STAGE_EX];
  switch(current_stage->cmd.opcode) {
    case CMD_ADD:
      current_stage->src1Val = current_stage->src1Val + current_stage->src2Val;
      break;
    case CMD_ADDI:
      current_stage->src1Val = current_stage->src1Val + current_stage->src2Val;
      break;
    case CMD_SUB:
      current_stage->src1Val = current_stage->src1Val - current_stage->src2Val;
      break;
    case CMD_SUBI:
      current_stage->src1Val = current_stage->src1Val - current_stage->src2Val;
      break;
  }
}

void DoWBStage() {
  PipeStageState* current_stage = &state.pipeStageState[STAGE_WB];

  if (state.pipeStageState[STAGE_WB].cmd.dst == 0) return;
  
  if (CMD_NOP < current_stage->cmd.opcode && CMD_SUBI >= current_stage->cmd.opcode) {
    state.regFile[current_stage->cmd.dst] = current_stage->src1Val;
  }
  
  else if (CMD_LOAD == current_stage->cmd.opcode) {
    state.regFile[current_stage->cmd.dst] = current_stage->src2Val;

    if (forwarding) {
      PipeStageState* ex_stage = &state.pipeStageState[STAGE_EX];

      if (ex_stage->cmd.src1 == current_stage->cmd.dst) {
        ex_stage->src1Val = state.regFile[current_stage->cmd.dst];
      }
      if (ex_stage->cmd.src2 == current_stage->cmd.dst &&
          !ex_stage->cmd.isSrc2Imm) {
        ex_stage->src2Val = state.regFile[current_stage->cmd.dst];
      }
    }
    
  }
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
  
  state.pc += 4;
  
  DoWBStage();
  /* TODO Continue Execute Stages */
  DoEXStage();
  DoIDStage();

  
  if (forwarding) {
    
  }
  else if (split_regfile) {
    
  }

  else { //Only Stalling
    for (int i = STAGE_WB; i > STAGE_IF; i--) { 
      state.pipeStageState[i] = state.pipeStageState[i-1];
    }
    bool data_hazard_detected = false;
    for (int i = STAGE_ID; i < SIM_PIPELINE_DEPTH; i ++){ 
      if (state.pipeStageState[i].cmd.opcode > CMD_NOP && state.pipeStageState[i].cmd.opcode <= CMD_LOAD) {
        int stage_dest = state.pipeStageState[i].cmd.dst;
        if (state.pipeStageState[STAGE_IF].cmd.src1 == stage_dest ||
            (!state.pipeStageState[STAGE_IF].cmd.isSrc2Imm) 
              && state.pipeStageState[STAGE_IF].cmd.src2 == stage_dest) {
              data_hazard_detected = true;
              break;         
        }
      }
    }
    if (data_hazard_detected) {
      reset_cmd(&state.pipeStageState[STAGE_ID].cmd); 
    }
    else {
      state.pipeStageState[STAGE_ID] = state.pipeStageState[STAGE_IF]; 
      SIM_MemInstRead(state.pc, &state.pipeStageState[STAGE_IF].cmd);
    }
  }
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
  memcpy(curState, &state, sizeof(SIM_coreState));
}

