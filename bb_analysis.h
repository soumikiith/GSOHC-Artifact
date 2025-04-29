#include "helper_func.h"

#define GPU_READ_WRITE 0

#define ONLY_DEVICE_TO_HOST 0

#define COMPILER_VERBOSE 0

bool gnk(llvm::Instruction *curr_instr, std::set<llvm::Value*> in_read_set, std::set<llvm::Value*> in_write_set, std::vector<mem_cont>in_bar,
        std::vector<mem_cont> &local_bar, std::set<llvm::Value*> &local_read, std::set<llvm::Value*> &local_write, bool &is_GPU_visited,
        std::vector<llvm::Instruction*> instr_stack, bool &kill)
{
    if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(curr_instr))
    {
        if(CI->getCalledFunction()->isIntrinsic())
        {
            kill = false;
            return false;
        }
        if(isNameEqual(CI->getCalledFunction()->getName(), "cudaDeviceSynchronize") || isNameEqual(CI->getCalledFunction()->getName(), "cudaFree") 
            || isNameEqual(CI->getCalledFunction()->getName(), "cudaThreadSynchronize"))
        {
            if(is_GPU_visited)
            {
                mem_cont curr_mem_cont;
                curr_mem_cont.instr = curr_instr;
                curr_mem_cont.context_arr = instr_stack;
                local_bar.push_back(curr_mem_cont); //Insert the current instruction in the In Map
            }
            kill = false;
            return false;
        }
        else if(isNameEqual(CI->getCalledFunction()->getName(), "cudaMemcpy"))
        {
            #if COMPILER_VERBOSE
                llvm::errs()<<"Line Memcpy: "<<CI->getDebugLoc().getLine()<<"\n";//debug
            #endif
            // llvm::errs()<<"Line Memcpy: "<<CI->getDebugLoc().getLine()<<" "<<is_GPU_visited<<"\n";//debug
            if(is_GPU_visited)
            {
                llvm::Value *last_val = CI->getArgOperand(3);
                if(llvm::ConstantInt *c_int = llvm::dyn_cast<llvm::ConstantInt>(last_val))
                {
                    #if ONLY_DEVICE_TO_HOST
                    if(c_int->getValue().getSExtValue()==2)
                    {
                    #endif
                        insert_local_rw(CI->getArgOperand(1), CI->getArgOperand(0), local_read, local_write, instr_stack);
                        mem_cont obj; //Creating a memory container object
                        obj.instr = curr_instr; //Inserting the instruction
                        obj.context_arr = instr_stack; //Inserting the context array
                        if(std::find(local_bar.begin(), local_bar.end(), obj) == local_bar.end())
                        {
                            mem_cont curr_mem_cont;
                            curr_mem_cont.instr = curr_instr;
                            curr_mem_cont.context_arr = instr_stack;
                            local_bar.push_back(curr_mem_cont); //insert into the local barrier set
                        }
                    #if ONLY_DEVICE_TO_HOST
                    }
                    #endif
                }
            }
            kill =false;
            return false;
        }
        else if(isGPUFunctionCall(curr_instr)){
            is_GPU_visited = true; //Mark the GPU function as visited
            #if GPU_READ_WRITE
            llvm::Function *gpu_func = CI->getCalledFunction()->getParent()->getFunction(get_device_func_name(CI->getCalledFunction()->getName().str()));
            #else
            llvm::Function *gpu_func = CI->getCalledFunction();
            #endif
            for(auto args = gpu_func->arg_begin(); args!=gpu_func->arg_end(); args++)
            {
                if(llvm::Argument *arg = llvm::dyn_cast<llvm::Argument>(args))
                {
                    int argument_index = arg->getArgNo();
                    llvm::Value *arg_val = get_true_value(CI->getArgOperand(argument_index), instr_stack);
                    #if GPU_READ_WRITE
                    if(isBeingRead(CI, param_rw_stat, argument_index, true))
                    {
                        // errs()<<*arg_val<<"\n";//debug
                        if(std::find(in_write_set.begin(), in_write_set.end(), arg_val) != in_write_set.end())
                        {
                            generate_memcpy_map(in_bar, kill, curr_instr, instr_stack);
                            return false;
                        }
                    }
                    else
                    {
                        if(std::find(in_write_set.begin(), in_write_set.end(), arg_val) != in_write_set.end() || 
                            std::find(in_read_set.begin(), in_read_set.end(), arg_val) != in_read_set.end())
                        {
                            generate_memcpy_map(in_bar, kill, curr_instr, instr_stack);
                            return false;
                        }
                    }
                    #else
                        if(std::find(in_write_set.begin(), in_write_set.end(), arg_val) != in_write_set.end() || 
                            std::find(in_read_set.begin(), in_read_set.end(), arg_val) != in_read_set.end())
                        {
                            generate_memcpy_map(in_bar, kill, curr_instr, instr_stack);
                            return false;
                        }
                    #endif
                }
            }
            return false;
        }
        else
        {
            llvm::Function *callee = CI->getCalledFunction();
            if(callee==nullptr)
            {
                kill = false;
                return false;
            }
            bool isArg_fact = isArgFact(CI, in_read_set, in_write_set, instr_stack); //Check if the function argument is a data flow fact
            if(callee->isDeclaration())
            {
                for(auto args = callee->arg_begin(); args!=callee->arg_end(); args++)
                {
                    int argument_index = args->getArgNo();
                    llvm::Value *arg_val = get_true_value(CI->getArgOperand(argument_index), instr_stack);
                    if(std::find(in_write_set.begin(), in_write_set.end(), arg_val) != in_write_set.end() || 
                        std::find(in_read_set.begin(), in_read_set.end(), arg_val) != in_read_set.end())
                    {
                        generate_memcpy_map(in_bar, kill, curr_instr, instr_stack);
                        return false;
                    }
                }
            }
            else if(gpu_containing_func.find(callee)!=gpu_containing_func.end() || isArg_fact)
            {
                inter_in_read_set[CI] = in_read_set; //perform insertion to the map
                inter_in_write_set[CI] = in_write_set; //perform insertion to the map
                inter_in_bar_set[CI] = in_bar; //perform insertion to the map
                inter_in_isGPUCall[CI] = is_GPU_visited; //perform insertion to the map
                return true; //True for Recurse
            }
            else{
                kill = false;
                return false;
            }    
        }
    }
    else if(llvm::LoadInst *LI = llvm::dyn_cast<llvm::LoadInst>(curr_instr))
    {
        //Identify if the load Instructions are used for passing arguments to the functions or not
        if(isLoadPassedArg(LI))
        {
            return false;
        }
        llvm::Value *load_ptr = get_true_value(LI->getPointerOperand(), instr_stack);
        if(std::find(in_write_set.begin(), in_write_set.end(), load_ptr) != in_write_set.end())
        {
            generate_memcpy_map(in_bar, kill, curr_instr, instr_stack);
            return false;
        }
    }
    else if(llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(curr_instr))
    {
        llvm::Value *store_ptr = get_true_value(SI->getPointerOperand(), instr_stack);
        if(std::find(in_write_set.begin(), in_write_set.end(), store_ptr) != in_write_set.end() || 
            std::find(in_read_set.begin(), in_read_set.end(), store_ptr) != in_read_set.end())
        {
            generate_memcpy_map(in_bar, kill, curr_instr, instr_stack);
            return false;
        }
    }
    else if(llvm::ReturnInst *RI = llvm::dyn_cast<llvm::ReturnInst>(curr_instr))
    {
        if(instr_stack.empty())
        {
            kill = true;
            return false;
        }
        else{
            //need to change the return value to the function call
            llvm::Instruction *callee_instr = instr_stack.back();
            if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(callee_instr))
            {
                inter_out_bar_set[CI] = {};
                inter_out_read_set[CI].insert(in_read_set.begin(), in_read_set.end());
                inter_out_write_set[CI].insert(in_write_set.begin(), in_write_set.end());
                for(auto iter = in_bar.begin(); iter != in_bar.end(); iter++)
                {
                    inter_out_bar_set[CI].push_back(*iter);
                }
                inter_out_isGPUCall[CI] = is_GPU_visited;
            }
            
            return false;
        }
    }
}


//Do iterate to find out suitable instruction
llvm::Instruction* find_suitable_instr(llvm::Instruction *memcpy_instr, llvm::Instruction *target_instr, llvm::DominatorTree *dom, 
            llvm::PostDominatorTree *pdom, llvm::LoopInfo* loopInfo)
{
    std::vector<llvm::Instruction*> worklist; //worklist to go by successors of the instruction
    std::set<llvm::Instruction*>visited_instr; //visited instructions
    std::vector<llvm::Instruction*> succ_instructions = getSuccessorInstructions(memcpy_instr);
    llvm::Instruction *return_target_instr = nullptr;
    for(auto iter = succ_instructions.begin(); iter != succ_instructions.end(); iter++)
    {
        worklist.push_back(*iter);
    }
    //Get the basic block of the memcpy instruction
    llvm::BasicBlock *memcpy_bb = memcpy_instr->getParent();
    for(auto iterator = memcpy_bb->begin(); iterator!=memcpy_bb->end(); iterator++)
    {
        if(llvm::Instruction *instr = llvm::dyn_cast<llvm::Instruction>(iterator))
        {
            if(instr == memcpy_instr)
                break;
            visited_instr.insert(instr); //insert the memcpy instruction in the visited set       
        }
    }
    while(!worklist.empty())
    {
        bool worklist_break = false;
        llvm::Instruction *front_instr = worklist.front();
        visited_instr.insert(front_instr);
        if(isSameLoopOrPDT(memcpy_instr, front_instr, dom, pdom, loopInfo))
        {
            if(dom->dominates(front_instr, target_instr) || pdom->dominates(target_instr, front_instr))
            {
                return_target_instr = front_instr;
            }
        }   
        succ_instructions = getSuccessorInstructions(front_instr);
        for(auto iter = succ_instructions.begin(); iter != succ_instructions.end(); iter++)
        {
            bool found = (visited_instr.find(*iter) != visited_instr.end());
            if(*iter == target_instr)
            {
                worklist_break = true;
                break;
            }
            if(!found)
            {
                worklist.push_back(*iter);
            }
        }
        if(worklist_break)
        {
            break;
        }
        worklist.erase(worklist.begin()); //delete the front of worklist
    }

    if(return_target_instr==nullptr)
    {
        #if COMPILER_VERBOSE
            llvm::errs()<<"SUITABLE INSTRUCTION"<<memcpy_instr->getDebugLoc().getLine()<<" "<<target_instr->getDebugLoc().getLine()<<"\n";//debug
        #endif
        worklist.clear(); //clear the worklist for furthe computation of nullptr target instructions
        succ_instructions.clear(); //clear the successor instructions
        visited_instr.clear(); //clear the visited instructions
        succ_instructions = getSuccessorInstructions(memcpy_instr);
        for(auto iter = succ_instructions.begin(); iter != succ_instructions.end(); iter++)
        {
            worklist.push_back(*iter);
        }
        while(!worklist.empty())
        {
            llvm::Instruction *front_instr = worklist.front();
            visited_instr.insert(front_instr);
            //patch to find the suitable instruction
            if(isSameLoopOrPDT(memcpy_instr, front_instr, dom, pdom, loopInfo) && memcpy_instr->getParent()!=front_instr->getParent())
            {
                // llvm::errs()<<memcpy_instr->getDebugLoc().getLine()<<" == "<<front_instr->getDebugLoc().getLine()<<"\n";//debug
                return_target_instr = front_instr;
            }
            succ_instructions = getSuccessorInstructions(front_instr);
            for(auto iter = succ_instructions.begin(); iter != succ_instructions.end(); iter++)
            {
                bool found = (visited_instr.find(*iter) != visited_instr.end());
                if(!found)
                {
                    worklist.push_back(*iter);
                }
            }
            worklist.erase(worklist.begin()); //delete the front of worklist
        }
    }
    return return_target_instr;
}

//Do iterate over refined statements to find out the suitable unified instruction
mem_cont find_unified_instr(std::vector<mem_cont> refined_set)
{
    mem_cont unified_instr;
    if(refined_set.size() == 1)
    {
        unified_instr.instr = refined_set[0].instr;
        unified_instr.context_arr = refined_set[0].context_arr;
        return unified_instr;
    }
    else if(ifAllSame(refined_set))
    {
        unified_instr.instr = refined_set[0].instr;
        unified_instr.context_arr = refined_set[0].context_arr;
        return unified_instr;
    }
    for(auto iter = refined_set.begin(); iter != refined_set.end(); iter++)
    {
        bool flag_dom = true;
        for(auto iter_j = refined_set.begin(); iter_j != refined_set.end(); iter_j++)
        {
            if(iter == iter_j)
                continue;
            else if(iter->instr->getParent() == iter_j->instr->getParent())
            {
                if(!isInSuccSBB(iter->instr, iter_j->instr))
                {
                    flag_dom = false;
                    break;
                }
            }
            else if(iter->instr->getFunction() == iter_j->instr->getFunction())
            {
                auto dom = func_to_dom.at(iter->instr->getFunction());
                if(!dom->dominates(iter->instr, iter_j->instr))
                {
                    flag_dom = false;
                    break;
                }
            }
            else
            {
                if(iter->context_arr.size() == 0 || iter_j->context_arr.size() == 0)
                {
                    if(iter->context_arr.size()==0)
                    {
                        auto dom = func_to_dom.at(iter->instr->getFunction());
                        if(!dom->dominates(iter->instr, iter_j->context_arr[0]))
                        {
                            flag_dom = false;
                            break;
                        }
                    }
                    else
                    {
                        auto dom = func_to_dom.at(iter_j->instr->getFunction());
                        if(!dom->dominates(iter->context_arr[0], iter_j->instr))
                        {
                            flag_dom = false;
                            break;
                        }
                    }
                }
                else
                {
                    llvm::Function *common_caller = getCommonCaller(iter->context_arr, iter_j->context_arr);
                    auto dom = func_to_dom.at(common_caller);
                    llvm::Instruction *s_caller, *d_caller;
                    for(auto c_i = iter->context_arr.begin(); c_i != iter->context_arr.end(); c_i++)
                    {
                        if((*c_i)->getFunction() == common_caller)
                        {
                            s_caller = *c_i;
                            break;
                        }
                    }
                    for(auto c_i = iter_j->context_arr.begin(); c_i != iter_j->context_arr.end(); c_i++)
                    {
                        if((*c_i)->getFunction() == common_caller)
                        {
                            d_caller = *c_i;
                            break;
                        }
                    }
                    if(!dom->dominates(s_caller, d_caller))
                    {
                        flag_dom = false;
                        break;
                    }
                }
            }
        }
        if(flag_dom)
        {
            unified_instr.instr = iter->instr;
            unified_instr.context_arr = iter->context_arr;
            break;
        }
    }
    return unified_instr;
}