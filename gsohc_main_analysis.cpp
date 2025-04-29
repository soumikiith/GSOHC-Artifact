#include "bb_analysis.h"

#define SKIP_SET 1

#define CALLGRAPH 1

#define READ_WRITE 1

#define PRINT_OUTPUT_CALLGRAPH 0

#define COMPILER_VERBOSE 0

#define DUMP_SUCCESSOR 0

#define OUTPUT_AFTER_REFINE 0

#define OUTPUT_AFTER_UNIFY 0

#define FILE_OUTPUT 1

#define TRANSFORM 1

#define INTER_PROC_TRANSFORM 1
using namespace llvm;

namespace {

std::map<llvm::Function*, std::set<llvm::Function*>>global_callgraph; //Call Graph of the Module (Custom Made).
std::vector<llvm::Instruction*> instr_stack; //Make an instruction stack
//make CallGraph
struct CustomCallGraphWrapperPass{
    
    std::map<llvm::Function*, std::vector<llvm::Value*>>func_param; // To store the function parameters
    std::set<llvm::Function*>visited_func; //A set to contain already visited functions
    std::vector<llvm::Function*>callgraph_worklist; //A worklist to traverse the CallGraph 

    bool run(Module &M)
    {

        llvm::CallGraph cgw(M);

        llvm::Function *main_func = M.getFunction("main"); //Get the @main Function
        //Insert into the CallGrah the first ever function (@main)
        global_callgraph.insert(std::pair<llvm::Function*, std::set<llvm::Function*>>(main_func, {}));
        
        //Push back into the CallGraph, Vector to implement a Worklist
        callgraph_worklist.push_back(main_func);

        //Iterate over CallGraphs
        while(!callgraph_worklist.empty())
        {
            //get the first Function of the CallGraph Worklist
            llvm::Function *first_func_cg = *(callgraph_worklist.begin());
            visited_func.insert(first_func_cg); //Mark the Function as visited
            llvm::CallGraphNode *first_func_node = cgw[first_func_cg]; //Get the CallGraph Node

            if(global_callgraph.find(first_func_cg)==global_callgraph.end())
            {
                global_callgraph.insert(std::pair<llvm::Function*, std::set<llvm::Function*>>(first_func_cg,{})); //Check whether the first of worklist exists inside the Callgraph or not
            }
            //Check for Callee
            for(auto iterator = first_func_node->begin(); iterator!=first_func_node->end(); iterator++)
            {
                llvm::CallGraphNode *callee_func_node = iterator->second;
                llvm::Function *callee_func_cg = callee_func_node->getFunction();

                //We do not care about the functions which are APIs, but not user-defined
                if(callee_func_cg==NULL || callee_func_cg->isDeclaration())
                {
                    continue;
                }

                //Insert the Callee into the Call Graph
                global_callgraph.at(first_func_cg).insert(callee_func_cg);

                //If already visited do not push it back into the callGraph
                if(visited_func.find(callee_func_cg)==visited_func.end())
                {
                    callgraph_worklist.push_back(callee_func_cg);
                }
            } 
            callgraph_worklist.erase(callgraph_worklist.begin());
        }
        #if PRINT_OUTPUT_CALLGRAPH
            for(auto iterator=global_callgraph.begin(); iterator!=global_callgraph.end();iterator++)
            {
                errs()<<"Caller: "<<iterator->first->getName().str()<<"\n";
                errs()<<"Callee: \n";
                for(auto iterator_j = iterator->second.begin(); iterator_j != iterator->second.end(); iterator_j++)
                {
                    llvm::Function *callee_func_set = *iterator_j;
                    errs()<<callee_func_set->getName().str()<<"\n";
                }
            }
        #endif
        if(global_callgraph.empty())
        {
            return false;
        }

        return true;
    }
};


struct ReadWriteWrapperPass{
    #if DUMP_READWRITE
        M->dump();
    #endif

    bool run(Module &M)
    {
      //check if the global callgraph is built or not
        if(global_callgraph.empty())
        {
            errs()<<"ERROR: CALL GRAPH WAS NOT BUILT !!"<<"\n";
            return false;
        }
        //populate the stack 
        std::vector<llvm::Function*>function_stack;
        make_stack(function_stack, global_callgraph, M); // Build the Function stack using CallGraph
        make_rw_analysis(param_rw_stat, function_stack, M);
        return true;
    }
};

struct FunctionWrapperPass{
    bool run(llvm::Function *func, std::vector<llvm::Instruction*>instr_stack)
    {
        llvm::CallInst *callsite_callinst;
        std::map<llvm::Instruction*, std::set<llvm::Value*>> in_read_set; //Read Set
        std::map<llvm::Instruction*, std::set<llvm::Value*>> in_write_set; //Write Set
        std::map<llvm::Instruction*, std::vector<mem_cont>>in_bar_set; //Barrier Set
        std::map<llvm::Instruction*, std::set<llvm::Value*>>out_read_set; //Out Read Set
        std::map<llvm::Instruction*, std::set<llvm::Value*>>out_write_set; //Out Write Set
        std::map<llvm::Instruction*, std::vector<mem_cont>> out_bar_set; //Barrier Set Out
        std::map<llvm::Instruction*, bool> in_visited_GPU; //If GPU kernel call is visited
        std::map<llvm::Instruction*, bool> out_visited_GPU; //If GPU kernel call is visited
        // if(instr_stack.empty())//check if instruction stack is empty
        // {
        //     errs()<<"-----------------------------------------------------------------Instruction Stack empty: Inside function: "<<func->getName().str()<<"-----------------------------------------------------------\n";
        // }
        std::vector<llvm::Instruction*> worklist; //Worklist
        std::set<llvm::Instruction*>visited_instr; //Visited Instruction
        worklist.push_back(&func->getEntryBlock().front()); //Push the first instruction of the function into the worklist
        for(auto iterator = func->begin(); iterator!=func->end(); iterator++)
        {
            for(auto iterator_j = iterator->begin(); iterator_j!=iterator->end(); iterator_j++)
            {
                llvm::Instruction *iterator_instr = &*iterator_j;
                generate_sets(iterator_instr, in_read_set, in_write_set, in_bar_set, instr_stack); //Generate the In sets w.r.t to current instruction with empty data
                generate_sets(iterator_instr, out_read_set, out_write_set, out_bar_set, instr_stack); //Generate the Out sets w.r.t to current instruction with empty data
                in_visited_GPU[iterator_instr] = false;//Mark the current instruction, where GPU kernel is not visited
                out_visited_GPU[iterator_instr] = false;//Mark the current instruction, where GPU kernel is not visited
            }
        }
        if(instr_stack.empty() == false) //Check if the instruction stack is empty or not
        {
            // errs()<<"-----------------------------------------------------------------Instruction Stack not empty: Inside function: "<<func->getName().str()<<"-------------------------------------------------------\n";
            llvm::Instruction *callsite_instr = instr_stack.back();
            if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(callsite_instr))
            {
                callsite_callinst = CI;
            }
            //Putting the initial In map into the start node of the calling function
            in_read_set[&func->getEntryBlock().front()] = inter_in_read_set.at(callsite_callinst); 
            in_write_set[&func->getEntryBlock().front()] = inter_in_write_set.at(callsite_callinst);
            in_bar_set[&func->getEntryBlock().front()] = inter_in_bar_set.at(callsite_callinst);
            in_visited_GPU[&func->getEntryBlock().front()] = inter_in_isGPUCall.at(callsite_callinst);
        }
        while(!worklist.empty())
        {
            bool kill = false;
            llvm::Instruction *curr_instr = *(worklist.begin()); //get the first instruction from the worklist
            //Define the Local Sets   
            std::vector<mem_cont>local_barrier_set;
            std::set<llvm::Value*>local_read_set;
            std::set<llvm::Value*>local_write_set;
            bool is_GPU_visited = in_visited_GPU.at(curr_instr); //Indicates if GPU is already visited or not
            visited_instr.insert(curr_instr); //Mark the current instruction as visited
            bool isRecurse = gnk(curr_instr, in_read_set.at(curr_instr), in_write_set.at(curr_instr), in_bar_set.at(curr_instr), 
            local_barrier_set, local_read_set, local_write_set, is_GPU_visited, instr_stack, kill); //Call the gnk function
            if(isRecurse)
            {
                if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(curr_instr))
                {
                    llvm::Function *callee_func = CI->getCalledFunction();
                    instr_stack.push_back(curr_instr); //Push the current instruction into the instruction stack
                    run(callee_func, instr_stack); //Recursively call the function for the new function
                    instr_stack.pop_back(); //Pop the current instruction from the instruction stack
                    local_read_set = inter_out_read_set.at(CI); //Get the Out Read Set
                    local_write_set = inter_out_write_set.at(CI); //Get the Out Write Set
                    local_barrier_set = inter_out_bar_set.at(CI); //Get the Out Barrier Set
                    is_GPU_visited = inter_out_isGPUCall.at(CI); //Get the Out GPU Visited Set

                    // print_set(local_write_set);//Debug
                    kill=true; //Set the kill flag to true as the function is recursive
                }
            }
            if(kill)
            {
                out_read_set.at(curr_instr) = local_read_set;
                out_write_set.at(curr_instr) = local_write_set;
                out_bar_set.at(curr_instr) = local_barrier_set;
            }
            else
            {
                unify_set(out_read_set.at(curr_instr), local_read_set, in_read_set.at(curr_instr));
                unify_set(out_write_set.at(curr_instr), local_write_set, in_write_set.at(curr_instr));
                unify_vector(out_bar_set.at(curr_instr), local_barrier_set, in_bar_set.at(curr_instr));
            }
            out_visited_GPU.at(curr_instr) = is_GPU_visited;
            
            //Current Node is visited, we need to visit the successors, by the rule of data flow we will flow the informations from the preds to the successors
            std::vector<llvm::Instruction*>successor_instr = getSuccessorInstructions(curr_instr); //Get the successor of the current instruction
            //insert instructions into the worklist
            #if DUMP_SUCCESSOR
                errs()<<"Successor of: "<<*curr_instr<<"-------------------\n";
                for(auto iterator = successor_instr.begin(); iterator!=successor_instr.end(); iterator++)
                {
                    errs()<<**iterator<<"\n";
                }
            #endif
            for(auto iterator=successor_instr.begin(); iterator!=successor_instr.end(); iterator++)
            {
                std::set<llvm::Value*>successor_read_set;//Read Set for successors
                std::set<llvm::Value*>successor_write_set;//Write Set for successors
                std::vector<mem_cont>successor_barrier_set;//Barrier Set for successors
                bool successor_is_GPU_visited = false;//Indicates if GPU is already visited or not

                std::vector<llvm::Instruction*>predecessor_instr = getPredecessorInstructions(*iterator); //Get the predecessor of the current instruction
                bool isPred_skipped_block = false; //Flag to check whether the predecessor is a skipped block or not
                llvm::BasicBlock *pred_skipped_bb; //Skipped Basic Block among the predecessor
                for(auto iterator_j = predecessor_instr.begin(); iterator_j!=predecessor_instr.end(); iterator_j++)
                {
                    successor_read_set.insert(out_read_set.at(*iterator_j).begin(), out_read_set.at(*iterator_j).end());
                    successor_write_set.insert(out_write_set.at(*iterator_j).begin(), out_write_set.at(*iterator_j).end());
                    unify_vector(successor_barrier_set, out_bar_set.at(*iterator_j), successor_barrier_set);
                    if(out_visited_GPU.at(*iterator_j))
                    {
                        successor_is_GPU_visited = true; // Mark the successor as visited (Unification)
                    }
                    #if SKIP_SET
                    if(skipped_set.find((*iterator_j)->getParent())!=skipped_set.end()) //Check whether the predecessor is a skipped block or not
                    {
                        isPred_skipped_block = true;
                        pred_skipped_bb = (*iterator_j)->getParent();
                        // llvm::errs()<<"Skipped Block: "<<pred_skipped_bb->getName().str()<<"\n";//debug
                        break;
                    }
                    #endif
                }
                if(visited_instr.find(*iterator)==visited_instr.end() || 
                    set_changed(in_read_set.at(*iterator), successor_read_set) || set_changed(in_write_set.at(*iterator), successor_write_set)
                    || in_bar_set.at(*iterator)!=successor_barrier_set || in_visited_GPU.at(*iterator)!=successor_is_GPU_visited)
                {
                    in_read_set.at(*iterator) = successor_read_set;
                    in_write_set.at(*iterator) = successor_write_set;
                    in_bar_set.at(*iterator) = successor_barrier_set;
                    in_visited_GPU.at(*iterator) = successor_is_GPU_visited;
                    worklist.push_back(*iterator);
                    #if SKIP_SET
                    //Check whether the predecessor is a skipped block or not and whether the current block is not a skipped block
                    if(isPred_skipped_block && skipped_set.find((*iterator)->getParent())==skipped_set.end())
                    {
                        // llvm::errs()<<**iterator<<" --- Skipped Block: "<<pred_skipped_bb->getName().str()<<" "<<in_bar_set.at(*iterator).size()<<"\n";//debug
                        // llvm::errs()<<(*iterator)->getDebugLoc().getLine()<<"\n";//debug
                        in_read_set.at(*iterator) = out_read_set.at(pred_skipped_bb->getTerminator());
                        in_write_set.at(*iterator) = out_write_set.at(pred_skipped_bb->getTerminator());
                        in_bar_set.at(*iterator) = out_bar_set.at(pred_skipped_bb->getTerminator());
                        // llvm::errs()<<"Skipped Block: "<<pred_skipped_bb->getName().str()<<" "<<in_bar_set.at(*iterator).size()<<"\n";//debug
                    } 
                    #endif
                }
                
            }
            worklist.erase(worklist.begin()); //Erase the first instruction from the worklist
        }
    }    
};

struct ModuleWrapperPass{

    void func_identification(llvm::Module &M) //identify basic blocks and identify GPU-containing functions
    {
        for(auto func_iter = M.begin(); func_iter!=M.end(); func_iter++)
        {
            for(auto bb_iter = func_iter->begin(); bb_iter!=func_iter->end(); bb_iter++)
            {
                for(auto instr_iter = bb_iter->begin(); instr_iter!=bb_iter->end(); instr_iter++)
                {
                    if(llvm::Instruction *instr = llvm::dyn_cast<llvm::Instruction>(instr_iter))
                    {
                        if(isGPUFunctionCall(instr))
                        {
                            if(llvm::BasicBlock *BB = llvm::dyn_cast<llvm::BasicBlock>(instr->getParent()))
                            {
                                skipped_set.insert(BB);
                            }
                            gpu_containing_func.insert(&*func_iter);
                            break;
                        }
                    }
                    if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(instr_iter))
                    {
                        llvm::Function *called_func = CI->getCalledFunction();
                        if(called_func==NULL)
                        {
                            continue;
                        }
                        if(isNameEqual(CI->getCalledFunction()->getName(), "cudaMemcpy") 
                        || isNameEqual(CI->getCalledFunction()->getName(), "cudaDeviceSynchronize"))
                        {
                            gpu_containing_func.insert(&*func_iter);
                        }
                    }
                }
            }
        }
    }
    bool run(Module &M)
    {
        #if DUMP_MODULE
            M.dump();
        #endif
        
        FunctionWrapperPass *FWP = new FunctionWrapperPass();
        std::vector<llvm::Instruction*>function_instr_stack;
        FWP->run(M.getFunction("main"), function_instr_stack);
    }
};

//counts the number of terminator insturctions and as well count the number of callsites
struct TerminatorInstWrapperPass{
    void run(Module &M)
    {
        //count number of callsites and terminator instructions
        for(auto iter_f = M.begin(); iter_f!=M.end(); iter_f++)
        {
            for(auto iter_bb = iter_f->begin(); iter_bb != iter_f->end(); iter_bb++)
            {
                for(auto iter=iter_bb->begin(); iter!=iter_bb->end(); iter++)
                {
                    if(llvm::ReturnInst *RI = llvm::dyn_cast<llvm::ReturnInst>(iter))
                    {
                        if(llvm::Function *func = llvm::dyn_cast<llvm::Function>(iter_f))
                        {
                            func_to_return[func].push_back(RI);
                        }
                    }
                    if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(iter))
                    {
                        llvm::Function *callee_func = CI->getCalledFunction();
                        func_to_callsite[callee_func]++;
                        // func_call_site[callee_func].push_back(CI);
                    }
                }
            }
        }
    }
};

struct PostProcessPass{
    void run()
    {
        std::map<mem_cont, std::vector<mem_cont>> refine_map; //After Refine Map
        std::map<mem_cont, mem_cont> unified_map; //Unified Map
        std::map<mem_cont, std::vector<mem_cont>> opp_copy;
        // print_result_analysis(); //Print the result of the analysis//debug
        post_process_copy(opp_copy);

        for(auto iter = opp_copy.begin(); iter!=opp_copy.end(); iter++)
        {
            mem_cont memcpy_stmt = iter->first;
            llvm::Instruction* memcpy_instr = memcpy_stmt.instr;
            std::vector<llvm::Instruction*> memcpy_instr_stack = memcpy_stmt.context_arr; //context of memcpy instruction
            llvm::Instruction *memcpy_stack_last_instr = nullptr;
            if(memcpy_instr_stack.size() > 0)
            {
                memcpy_stack_last_instr = memcpy_instr_stack.back(); //get the last instruction of the memcpy instruction stack
            }
            for(auto iter_j = iter->second.begin(); iter_j!=iter->second.end(); iter_j++)
            {
                mem_cont target_stmt = *iter_j;
                llvm::Instruction *target_instr = target_stmt.instr;
                std::vector<llvm::Instruction*> target_instr_stack = target_stmt.context_arr; //context of target instruction
                llvm::Instruction *target_stack_last_instr = nullptr;
                if(target_instr_stack.size() > 0)
                {
                    target_stack_last_instr = target_instr_stack.back(); //get the last instruction of the target instruction stack
                }
                if(memcpy_stack_last_instr == nullptr && target_stack_last_instr == nullptr)
                {
                    //Memcpy Instruction and target instruction are in the main function
                    auto dom = func_to_dom.at(memcpy_instr->getFunction()); //get the dominator instance
                    auto pdom = func_to_pdom.at(memcpy_instr->getFunction()); //get the post-dominator instance
                    auto loopInfo = func_to_loop.at(memcpy_instr->getFunction()); //get the loop info instance

                    mem_cont target_obj; //Declaring target object as a mem_cont object
                    target_obj.context_arr = target_instr_stack; //Assigning the context of target instruction to the target object

                    if(memcpy_instr->getParent() == target_instr->getParent()) //source and target in same basic block
                    {
                        target_obj.instr = target_instr; //Assign the target instruction to the target object
                        // refine_map[memcpy_stmt].push_back(target_obj);
                    }
                    else if(isSameLoopOrPDT(memcpy_instr, target_instr, dom, pdom, loopInfo)) 
                    {
                        target_obj.instr = target_instr; //Assign the target instruction to the target object
                        // refine_map[memcpy_stmt].push_back(target_obj);
                    }
                    else
                    {
                        target_instr = find_suitable_instr(memcpy_instr, target_instr, dom, pdom, loopInfo);
                        // llvm::errs()<<"Source and Target in same basic block "<< memcpy_instr->getDebugLoc().getLine()<<" "<<
                        // target_instr->getDebugLoc().getLine()<<"\n";//debug
                        target_obj.instr = target_instr; //Assign the target instruction to the target object
                    }
                    refine_map[memcpy_stmt].push_back(target_obj);
                }
                else if((memcpy_instr->getFunction() == target_instr->getFunction())
                    && ( memcpy_stack_last_instr == target_stack_last_instr )) //source and target have same context info
                {
                    //Source and target are not in the same Function but not in main
                    auto dom = func_to_dom.at(memcpy_instr->getFunction()); //get the dominator instance
                    auto pdom = func_to_pdom.at(memcpy_instr->getFunction()); //get the post-dominator instance
                    auto loopInfo = func_to_loop.at(memcpy_instr->getFunction()); //get the loop info instance

                    mem_cont target_obj; //Declaring target object as a mem_cont object
                    target_obj.context_arr = target_instr_stack; //Assigning the context of target instruction to the target object

                    if(memcpy_instr->getParent() == target_instr->getParent()) //source and target in same basic block
                    {
                        target_obj.instr = target_instr; //Assign the target instruction to the target object
                        // refine_map[memcpy_stmt].push_back(target_obj);
                    }
                    else if(isSameLoopOrPDT(memcpy_instr, target_instr, dom, pdom, loopInfo)) 
                    {
                        target_obj.instr = target_instr; //Assign the target instruction to the target object
                        // refine_map[memcpy_stmt].push_back(target_obj);
                    }
                    else
                    {
                        target_instr = find_suitable_instr(memcpy_instr, target_instr, dom, pdom, loopInfo);
                        // llvm::errs()<<"Source and Target in same basic block "<< memcpy_instr->getDebugLoc().getLine()<<" "<<
                        // target_instr->getDebugLoc().getLine()<<"\n";//debug
                        target_obj.instr = target_instr; //Assign the target instruction to the target object
                    }
                    refine_map[memcpy_stmt].push_back(target_obj);
                }
                else { 
                    // source and target are in two different functions
                    if(memcpy_instr_stack.size() != 0 && target_instr_stack.size() != 0)
                    {
                        llvm::Instruction *intersect_instr = memcpy_target_intersect(memcpy_instr_stack, target_instr_stack);
                        if(intersect_instr != nullptr)
                        {
                            target_instr = intersect_instr;
                            //Remove till the intersect instruction from target instruction stack
                            std::vector<llvm::Instruction*> new_target_instr_stack;
                            for(auto iter_k = target_instr_stack.begin(); iter_k!=target_instr_stack.end(); iter_k++)
                            {
                                if(*iter_k == intersect_instr)
                                {
                                    break;
                                }
                                new_target_instr_stack.push_back(*iter_k);
                            }
                            target_instr_stack = new_target_instr_stack;//Trimmed target instruction stack
                        }
                    }

                    bool isSourceLevel; 
                    llvm::Instruction *next_callsite = nullptr;
                    llvm::Function *dest_func = findDestFunction(memcpy_instr, target_instr, memcpy_instr_stack, target_instr_stack, 
                        isSourceLevel, next_callsite);
                    // llvm::errs()<<"Next Callsite: "<<next_callsite->getDebugLoc()->getLine()<<"\n";//debug
                    mem_cont target_obj; //Declaring target object as a mem_cont object
                    if(isSourceLevel)
                        target_obj.context_arr = getContextDest(memcpy_instr_stack, dest_func); //Assigning the context of memcpy instruction to the target object
                    else
                        target_obj.context_arr = getContextDest(target_instr_stack, dest_func); //Assigning the context of target instruction to the target object
                    if(dest_func == memcpy_instr->getFunction()) //source and destiantion in same function
                    {
                        auto dom_p = func_to_dom.at(memcpy_instr->getFunction()); //get the dominator instance
                        auto pdom_p = func_to_pdom.at(memcpy_instr->getFunction()); //get the dominator instance
                        auto loopInfo_p = func_to_loop.at(memcpy_instr->getFunction()); //get the loop info instance
                        if(isSourceLevel){
                            std::vector<llvm::ReturnInst*> return_instr_set = func_to_return.at(dest_func); //get return instructions of the function
                            for(auto iter_k = return_instr_set.begin(); iter_k!=return_instr_set.end(); iter_k++)
                            {
                                target_instr = find_suitable_instr(memcpy_instr, *iter_k, dom_p, pdom_p, loopInfo_p); //find refined target locations from that particular function
                            }
                            target_obj.instr = target_instr; //Assign the target instruction to the target object
                            if(dest_func->getName().str() == "main")
                            {
                                target_obj.context_arr.clear();//the instructions in main should not have any context
                            }
                            refine_map[memcpy_stmt].push_back(target_obj); //store the refined target locations
                        }
                        else
                        {
                            llvm::Instruction *call_target = target_obj.context_arr.front();
                            target_instr = find_suitable_instr(memcpy_instr, call_target, dom_p, pdom_p, loopInfo_p); //find refined target locations from that particular function
                            target_obj.instr = target_instr; //Assign the target instruction to the target object
                            if(dest_func->getName().str() == "main")
                            {
                                target_obj.context_arr.clear();//the instructions in main should not have any context
                            }
                            refine_map[memcpy_stmt].push_back(target_obj); //store the refined target locations
                        }
                    }
                    else
                    {
                        llvm::Function *common_func = getCommonCaller(memcpy_instr_stack, target_instr_stack);
                        if(common_func==nullptr)
                        {
                            common_func = memcpy_instr->getFunction()->getParent()->getFunction("main"); //get Main function from module M
                        }
                        if(dest_func == common_func)
                        {
                            auto dom_p = func_to_dom.at(common_func); //get the dominator instance
                            auto pdom_p = func_to_pdom.at(common_func); //get the dominator instance
                            auto loopInfo_p = func_to_loop.at(common_func); //get the loop info instance
                            llvm::Instruction *common_memcpy_instr;//to store the source call site in the common caller function

                            for(auto iter_k = memcpy_instr_stack.begin(); iter_k!=memcpy_instr_stack.end(); iter_k++)
                            {
                                if((*iter_k)->getFunction() == common_func)
                                {
                                    common_memcpy_instr = *iter_k;
                                    break;
                                }
                            }
                            if(common_memcpy_instr->getParent() == target_instr->getParent()) //source and target in same basic block
                            {
                                target_obj.instr = target_instr; //Assign the target instruction to the target object
                            }
                            else if(isSameLoopOrPDT(common_memcpy_instr, target_instr, dom_p, pdom_p, loopInfo_p))
                            {
                                target_obj.instr = target_instr; //Assign the target instruction to the target object
                            }
                            else
                            {
                                target_instr = find_suitable_instr(common_memcpy_instr, target_instr, dom_p, pdom_p, loopInfo_p); //find refined target locations from that particular function
                                target_obj.instr = target_instr; //Assign the target instruction to the target object
                                if(dest_func->getName().str() == "main")
                                {
                                    target_obj.context_arr.clear();//the instructions in main should not have any context
                                }
                                else
                                {
                                    target_obj.context_arr = getContextDest(target_instr_stack, dest_func); //Assigning the context of target instruction to the target object
                                }
                            }
                            refine_map[memcpy_stmt].push_back(target_obj); //store the refined target locations
                        }
                        else{
                            auto dom_p = func_to_dom.at(next_callsite->getFunction()); //get the dominator instance
                            auto pdom_p = func_to_pdom.at(next_callsite->getFunction()); //get the dominator instance
                            auto loopInfo_p = func_to_loop.at(next_callsite->getFunction()); //get the loop info instance
                            llvm::Instruction *enter_instr = &dest_func->getEntryBlock().front(); //get the entry instruction of the function

                            target_instr = find_suitable_instr(enter_instr, next_callsite, dom_p, pdom_p, loopInfo_p); //find refined target locations from that particular function
                            target_obj.instr = target_instr; //Assign the target instruction to the target object
                            if(dest_func->getName().str() == "main")
                            {
                                target_obj.context_arr.clear();//the instructions in main should not have any context
                            }
                            refine_map[memcpy_stmt].push_back(target_obj); //store the refined target locations
                        }
                    }
                }
            }
        }
        //end of refinement
        printCustomMessage("Refinement successfully ended.", "success");
        //In the refinement, it may not find any suitable instruction also, in that case it should move to its current location only.
        for(auto iter = refine_map.begin(); iter!=refine_map.end(); iter++)
        {
            bool flag_all_nullptr = true;
            for(auto iter_j = iter->second.begin(); iter_j!=iter->second.end(); iter_j++)
            {
                if(iter_j->instr != nullptr)
                {
                    flag_all_nullptr = false;
                    break;
                }
            }
            if(flag_all_nullptr)
            {
                iter->second.clear();
                mem_cont obj;
                obj.instr = iter->first.instr;
                obj.context_arr = iter->first.context_arr;
                iter->second.push_back(obj);
            }
        }
        //If one entry is NULL, that means on that path, it was unable to find a suitable instruction, so we need to remove that entry
        for(auto iter = refine_map.begin(); iter!=refine_map.end(); iter++)
        {
            for(auto iter_j = iter->second.begin(); iter_j!=iter->second.end(); iter_j++)
            {
                if(iter_j->instr == nullptr)
                {
                    iter->second.erase(iter_j);
                }
            }
        }
        printCustomMessage("Starting unification...", "info");
        //start of unification
        for(auto iter = refine_map.begin(); iter!=refine_map.end(); iter++)
        {
            mem_cont target_instr = find_unified_instr(iter->second); //find the unified instruction
            unified_map[iter->first] = target_instr; //store the unified instruction
        }
        //end of unification
        printCustomMessage("Beginning to print analysis results...", "info");
        //start of function refinement
        for(auto iter=unified_map.begin(); iter!=unified_map.end(); iter++)
        {
            if(iter->second.context_arr.size()==0)
            {
                correct_map[iter->first] = iter->second;
                continue;
            }
            bool is_multiple_callsite = false;
            for(auto iter_j = iter->second.context_arr.begin(); iter_j!=iter->second.context_arr.end(); iter_j++)
            {
                if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(*iter_j))
                {
                    llvm::Function *func = CI->getCalledFunction();
                    if(func_to_callsite.at(func)>1)
                    {
                        is_multiple_callsite = true;
                    }
                }
            }
            if(is_multiple_callsite && iter->first.instr->getFunction() != iter->second.instr->getFunction())
            {
                for(auto iter_j = iter->second.context_arr.begin(); iter_j!=iter->second.context_arr.end(); iter_j++)
                {
                    if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(*iter_j))
                    {
                        llvm::Function *func = CI->getCalledFunction();
                        if(func_to_callsite.at(func)>1)
                        {
                            mem_cont obj;
                            obj.instr = *iter_j;
                            llvm::Function *callee_func = CI->getFunction();
                            obj.context_arr = getContextDest(iter->second.context_arr, callee_func);
                            correct_map[iter->first] = obj;
                        }
                    }
                }
                continue;
            }
            else
            {
                correct_map[iter->first] = iter->second;
            }
        }
        //store the instructions and contexts in a seperate map; where the instruction is same but the context is different
        //store the visited memcpy instructions and corresponding from how many contexts they
        // have been invoked
        std::map<llvm::Instruction*, int> visited_memcpy_mul_cont;
        for(auto iter = correct_map.begin(); iter!=correct_map.end(); iter++)
        {
            llvm::Instruction *memcpy_instr = iter->first.instr;
            if(visited_memcpy_mul_cont.find(memcpy_instr) == visited_memcpy_mul_cont.end())
            {
                visited_memcpy_mul_cont[memcpy_instr] = 1;
            }
            else
            {
                visited_memcpy_mul_cont[memcpy_instr]++;
            }
        }
        for(auto iter = correct_map.begin(); iter!=correct_map.end(); iter++)
        {
            llvm::Instruction *memcpy_instr = iter->first.instr;
            std::vector<llvm::Instruction*> memcpy_instr_stack = iter->first.context_arr;
            if(visited_memcpy_mul_cont.find(memcpy_instr) != visited_memcpy_mul_cont.end())
            {
                if(visited_memcpy_mul_cont[memcpy_instr] > 1)
                {
                    trans_instruction_mul_context.insert(std::pair<llvm::Instruction*, std::vector<llvm::Instruction*>>(memcpy_instr, memcpy_instr_stack));
                }
            }
        }
        visited_memcpy_mul_cont.clear();
        #if OUTPUT_AFTER_REFINE
            errs()<<"\n\n====================================================== Final REFINE Map: ===============================================\n";
            for(auto iter = refine_map.begin(); iter!=refine_map.end(); iter++)
            {
                errs()<<"Memcpy Line Number: "<<iter->first.instr->getDebugLoc().getLine()<<"--------------------->";
                for(auto iter_j = iter->second.begin(); iter_j!=iter->second.end(); iter_j++)
                {
                    errs()<<"Target Line Number: "<<iter_j->instr->getDebugLoc().getLine()<<"------------ AND ----------\n";
                }
                errs()<<"\n";
            }
        #endif
        #if OUTPUT_AFTER_UNIFY
        errs()<<"\n\n=============================================================================================================== Final UNIFIED Map: ===================================================================================================================\n";
        for(auto iter = unified_map.begin(); iter!=unified_map.end(); iter++)
        {
            errs()<<"Memcpy Line Number: "<<iter->first.instr->getDebugLoc().getLine()<<"--------------------->";
            errs()<<"Target Line Number: "<<iter->second.instr->getDebugLoc().getLine()<<"\n";
        }
        #endif
        // errs()<<"\n\n========================================== Final OUTPUT Map: ==================================================\n";
        printCustomMessage("Printing final output map...", "debug");
        // for(auto iter = correct_map.begin(); iter!=correct_map.end(); iter++)
        // {
        //     errs()<<"Memcpy Line Number: "<<iter->first.instr->getDebugLoc().getLine()<<"--------------------->";
        //     errs()<<"Target Line Number: "<<iter->second.instr->getDebugLoc().getLine()<<"\n";
        // }

//==================================================================== OUTPUT ==========================================================================

    // Header
        errs() << BLUE << "+-------------------------+\n";
        errs() << BLUE << "| " << YELLOW << "Memcpy" << BLUE << "  ⟿  " << GREEN << "Target Lines" << BLUE << " |\n";
        errs() << BLUE << "+-------------------------+\n";

        // Loop through mappings
        for(auto iter = correct_map.begin(); iter != correct_map.end(); iter++) {
            if(iter->first.instr == nullptr || iter->second.instr == nullptr) {
                continue; // Skip if either instruction is null
            }
            // Print the lines
            int memcpyLine = iter->first.instr->getDebugLoc().getLine();
            int targetLine = iter->second.instr->getDebugLoc().getLine();
            
            errs() << BLUE << "| " << YELLOW << memcpyLine 
                << BLUE << "  ⟿  " << GREEN << targetLine 
                << BLUE << "              |\n";
        }

        // Footer
        errs() << BLUE << "+-------------------------+" << RESET << "\n";
    }
//==================================================================== OUTPUT ==========================================================================

};

struct ChangeIRWrapperPass{
    void run(llvm::Module &M)
    {
        std::map<llvm::Instruction*, std::vector<llvm::Value*>> func_to_param;

        std::map<llvm::Function*, std::vector<llvm::Type*>> func_to_param_type; //store the parameters' type of the memcpy function
        std::map<llvm::Function*, std::vector<llvm::Value*>> func_to_param_value; //store the parameters' value of the memcpy function
        
        std::map<llvm::Function*, ValueToValueMapTy*> FToVMap; //store the value to value map

        //create a new function with same parameters as parent function of memcpy instruction
        std::set<llvm::Instruction*> visited_memcpy; //store the visited memcpy instructions
        std::set<llvm::Function*> visited_target_func; //store the visited target functions

        std::map<mem_cont, mem_cont> inter_proc_correct_map;
        printCustomMessage("Starting transformation...", "info"); //compiler verbose

        //Intra Procedural Transformation - Move Memcpy and cudaThreadSynchronize
        for(auto iter = correct_map.begin(); iter!=correct_map.end(); iter++)
        {
            llvm::Instruction *memcpy_instr = iter->first.instr;
            llvm::Instruction *target_instr = iter->second.instr;
            std::vector<llvm::Instruction*>memcpy_instr_stack = iter->first.context_arr;
            std::vector<llvm::Instruction*>target_instr_stack = iter->second.context_arr;

            //Intra Procedural Transformation
            //First transfer the memcpy instruction before the target instruction, then cudaFree at the last
            if(memcpy_instr->getFunction() == target_instr->getFunction())
            {
                if(getFunctionFromInstr(memcpy_instr) && 
                    (isNameEqual(getFunctionFromInstr(memcpy_instr)->getName(), "cudaMemcpy")||
                    isNameEqual(getFunctionFromInstr(memcpy_instr)->getName(), "cudaDeviceSynchronize")||
                    isNameEqual(getFunctionFromInstr(memcpy_instr)->getName(), "cudaThreadSynchronize")))
                {
                    memcpy_instr->moveBefore(target_instr); //moving memcpy instruction before the target instruction
                }
            }
        }
        //Intra Procedural Transformation - Move cudaFree
        for(auto iter = correct_map.begin(); iter!=correct_map.end(); iter++)
        {
            llvm::Instruction *memcpy_instr = iter->first.instr;
            llvm::Instruction *target_instr = iter->second.instr;
            std::vector<llvm::Instruction*>memcpy_instr_stack = iter->first.context_arr;
            std::vector<llvm::Instruction*>target_instr_stack = iter->second.context_arr;

            //Intra Procedural Transformation
            //First transfer the memcpy instruction before the target instruction, then cudaFree at the last
            if(memcpy_instr->getFunction() == target_instr->getFunction())
            {
                if(getFunctionFromInstr(memcpy_instr) &&  isNameEqual(getFunctionFromInstr(memcpy_instr)->getName(), "cudaFree"))
                {
                    memcpy_instr->moveBefore(target_instr); //moving memcpy instruction before the target instruction
                    // correct_map.erase(iter); //erase the current entry from the correct map
                }
            }
        }

        printCustomMessage("Intra-Procedural transformation completed.", "success"); //compiler verbose

        //Inter Procedural Transformation
        bool flag_inter_trans_print = true;
        for(auto iter = correct_map.begin(); iter!=correct_map.end(); iter++)
        {
            #if INTER_PROC_TRANSFORM
            llvm::Instruction *memcpy_instr = iter->first.instr;
            llvm::Instruction *target_instr = iter->second.instr;
            std::vector<llvm::Instruction*>memcpy_instr_stack = iter->first.context_arr;
            std::vector<llvm::Instruction*>target_instr_stack = iter->second.context_arr;

            if(memcpy_instr->getFunction() != target_instr->getFunction())
            {   
                if(flag_inter_trans_print)
                {
                    printCustomMessage("Inter-Procedural transformation starting...", "info");
                    flag_inter_trans_print = false;
                }
                inter_proc_correct_map[iter->first] = iter->second;
                
                std::vector<llvm::Instruction*> source_contextual_path = memcpy_instr_stack;
                source_contextual_path.push_back(memcpy_instr);
                std::vector<llvm::Instruction*> target_contextual_path = target_instr_stack;
                target_contextual_path.push_back(target_instr);

                llvm::Function *common_Caller = getCommonCaller(source_contextual_path, target_contextual_path);
                int com_index = getCommonCallerIndex(source_contextual_path, target_contextual_path);

                if(memcpy_instr->getFunction() != common_Caller && target_instr->getFunction() != common_Caller)
                {
                    if(visited_memcpy.find(memcpy_instr) == visited_memcpy.end())
                    {
                        memcpy_transform(memcpy_instr, func_to_param_type, func_to_param_value);
                        visited_memcpy.insert(memcpy_instr);
                    }
                        // memcpy_transform(target_instr, mempcy_func_to_param_type, mempcy_func_to_param_value);
                    //for each target we have to perform this
                    mem_cont memcpy_instr_cont = iter->first;//get the struct for the memcpy instruction from the correct map
                    mem_cont target_instr_cont = iter->second;//get the struct for the target instruction from the correct map
                    target_transform(memcpy_instr_cont, target_instr_cont, func_to_param_type, func_to_param_value, target_instr, visited_target_func);
                    visited_memcpy.insert(target_instr);
                }
                else if(memcpy_instr->getFunction() == common_Caller && target_instr->getFunction() != common_Caller)
                {
                    visited_memcpy.insert(memcpy_instr);
                    mem_cont memcpy_instr_cont = iter->first;//get the struct for the memcpy instruction from the correct map
                    mem_cont target_instr_cont = iter->second;//get the struct for the target instruction from the correct map
                    target_transform(memcpy_instr_cont, target_instr_cont, func_to_param_type, func_to_param_value, target_instr, visited_target_func);
                    visited_memcpy.insert(target_instr);
                }
                else if(memcpy_instr->getFunction() != common_Caller && target_instr->getFunction() == common_Caller)
                {
                    if(visited_memcpy.find(memcpy_instr) == visited_memcpy.end())
                    {   
                        memcpy_transform(memcpy_instr, func_to_param_type, func_to_param_value);
                        visited_memcpy.insert(memcpy_instr);
                    }
                    mem_cont memcpy_instr_cont = iter->first;//get the struct for the memcpy instruction from the correct map
                    visited_memcpy.insert(target_instr);
                }
            }
            #endif
        }
        
        std::map<llvm::Instruction*, llvm::Instruction*> original_sync_to_cloned; // map from original instructions to cloned instructions: ORIG -> CLONED

        clone_func(inter_proc_correct_map, func_to_param_type, func_to_param_value, M);

        printCustomMessage("Function definition modified.", "debug");

        clone_call_sites(inter_proc_correct_map, func_to_param_value, M);

        printCustomMessage("Callsites have been modified.", "debug");

        
        std::set<llvm::Instruction*> processed_instr; //To store the processed insturctions
        
        for(auto iter = inter_proc_correct_map.begin(); iter!=inter_proc_correct_map.end(); iter++)
        {
            llvm::Instruction *target_instr = iter->second.instr; //get the target instruction
            llvm::Instruction *memcpy_instr = iter->first.instr; //get the memcpy instruction

            std::vector<llvm::Instruction*> target_context = iter->second.context_arr; //get the target context
            std::vector<llvm::Instruction*> src_context = iter->first.context_arr; //get the source context

            llvm::CallInst *target_call_site;
            if(target_context.size() != 0)
            {
                llvm::Instruction *target_call_instr = target_context.back(); //get the last instruction of the target context
                if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(target_call_instr))
                {
                    target_call_site = CI;
                }
            }

            llvm::Function *common_caller = getCommonCaller(src_context, target_context); //get the common caller function
            if(common_caller == nullptr)
            {
                common_caller = M.getFunction("main"); //get the main function
            }
            if(memcpy_instr->getFunction() != common_caller && target_instr->getFunction()!=common_caller)
            {
                llvm::Instruction *target_instr_clone = map_to_cloned_instr.at(target_instr); //get the cloned target instruction

                llvm::Function *target_func = target_instr->getFunction(); //get the target function

                if(llvm::CallInst *CI_mem = llvm::dyn_cast<llvm::CallInst>(memcpy_instr))
                {
                    std::vector<llvm::Value*> arg_val = {};
                    if(CI_mem->getCalledFunction()->getName().str()=="cudaDeviceSynchronize" || 
                    CI_mem->getCalledFunction()->getName().str()=="cudaThreadSynchronize")
                    {
                        llvm::Instruction *target_instr = iter->second.instr;
                        llvm::Instruction *target_instr_clone = map_to_cloned_instr.at(target_instr);
                        // std::vector<llvm::Value*> arg_val;
                        llvm::CallInst *sync_clone = llvm::CallInst::Create(CI_mem->getCalledFunction(), arg_val, "call_sync.gsohc_", target_instr_clone);
                        sync_clone->setDebugLoc(target_instr_clone->getDebugLoc());
                    }
                    
                    else if(CI_mem->getCalledFunction()->getName().str()=="cudaMemcpy")
                    {
                        // std::vector<llvm::Value*> arg_val;

                        llvm::Value *val_mem_f = get_true_value(CI_mem->getArgOperand(0));
                        llvm::Value *val_mem_s = get_true_value(CI_mem->getArgOperand(1));
                        llvm::BitCastInst *bitcast_val_f, *bitcast_val_s;
                        llvm::Value *val_f, *val_s;
                        if(!isa<llvm::Argument>(val_mem_f))
                        {
                            val_f = FuncToVMap.at(target_func).lookup(val_mem_f);
                        }
                        else
                        {
                            val_mem_f = get_true_value(CI_mem->getArgOperand(0), src_context);
                            int index = 0;
                            for(auto arg_iter = target_call_site->arg_begin(); arg_iter!=target_call_site->arg_end(); arg_iter++)
                            {
                                if((*arg_iter)->getType()->isPointerTy())
                                {
                                    llvm::Value *arg_true_val = get_true_value(*arg_iter);
                                    if(arg_true_val == val_mem_f)
                                    {
                                        val_f = func_to_cloned_func.at(target_func)->getArg(index);
                                        
                                    }
                                }
                                index++;
                            }
                            // val_f = FuncToVMap.at(target_func).lookup(val_mem_f);
                        }
                        if(!isa<llvm::Argument>(val_mem_s))
                        {
                            val_s = FuncToVMap.at(target_func).lookup(val_mem_s);
                        }
                        else
                        {
                            val_mem_s = get_true_value(CI_mem->getArgOperand(1), src_context);
                            val_s = FuncToVMap.at(target_func).lookup(val_mem_s);
                        }
                        if(isPointerToPointer(val_f))
                        {
                            llvm::LoadInst *load_val_f = new llvm::LoadInst(val_f->getType()->getPointerElementType(),  val_f, "load.gsohc_", target_instr_clone);
                            bitcast_val_f = new llvm::BitCastInst(load_val_f, llvm::Type::getInt8PtrTy(M.getContext()), "bitcast_mem.gsohc_", target_instr_clone);

                        }
                        else
                        {
                            bitcast_val_f = new llvm::BitCastInst(val_f, llvm::Type::getInt8PtrTy(M.getContext()), "bitcast_mem.gsohc_", target_instr_clone);
                        }
                        if(isPointerToPointer(val_s))
                        {
                            llvm::LoadInst *load_val_s = new llvm::LoadInst(val_s->getType()->getPointerElementType(),  val_s, "load.gsohc_", target_instr_clone);
                            bitcast_val_s = new llvm::BitCastInst(load_val_s, llvm::Type::getInt8PtrTy(M.getContext()), "bitcast_mem.gsohc_", target_instr_clone);
                        }
                        else
                        {
                            bitcast_val_s = new llvm::BitCastInst(val_s, llvm::Type::getInt8PtrTy(M.getContext()), "bitcast_mem.gsohc_", target_instr_clone);
                        }

                        arg_val.push_back(bitcast_val_f);
                        arg_val.push_back(bitcast_val_s);
                        arg_val.push_back(CI_mem->getArgOperand(2));
                        arg_val.push_back(CI_mem->getArgOperand(3));
                        llvm::CallInst *copy_mem = llvm::CallInst::Create(CI_mem->getCalledFunction(), arg_val, "call_mem.gsohc_", target_instr_clone); 
                        copy_mem->setDebugLoc(target_instr_clone->getDebugLoc());

                    }
                    processed_instr.insert(memcpy_instr);
                }
            }
            else if(memcpy_instr->getFunction() != common_caller && target_instr->getFunction() == common_caller)
            {
                llvm::Function *target_func = target_instr->getFunction(); //get the target function
                llvm::Function *memcpy_func = memcpy_instr->getFunction(); //get the memcpy function

                if(llvm::CallInst *CI_mem = llvm::dyn_cast<llvm::CallInst>(memcpy_instr))
                {
                    std::vector<llvm::Value*> arg_val = {};
                    if(CI_mem->getCalledFunction()->getName().str()=="cudaDeviceSynchronize" || 
                    CI_mem->getCalledFunction()->getName().str()=="cudaThreadSynchronize")
                    {
                        llvm::Instruction *target_instr = iter->second.instr;
                        // std::vector<llvm::Value*> arg_val;
                        llvm::CallInst *sync_clone = llvm::CallInst::Create(CI_mem->getCalledFunction(), arg_val, "call_sync.gsohc_", target_instr);
                        sync_clone->setDebugLoc(target_instr->getDebugLoc());
                        map_to_cloned_instr[CI_mem] = sync_clone;
                    }
                    
                    else if(CI_mem->getCalledFunction()->getName().str()=="cudaMemcpy")
                    {
                        // std::vector<llvm::Value*> arg_val;
                        llvm::Value *val_mem_f = get_true_value(CI_mem->getArgOperand(0));
                        llvm::Value *val_mem_s = get_true_value(CI_mem->getArgOperand(1));
                        
                        llvm::BitCastInst *bitcast_val_f, *bitcast_val_s;
                        llvm::Value *val_f, *val_s;
                        if(!isa<llvm::Argument>(val_mem_f))
                        {
                            val_f = FuncToVMap.at(target_func).lookup(val_mem_f);
                        }
                        else
                        {
                            val_mem_f = get_true_value(CI_mem->getArgOperand(0), src_context);
                            val_f = val_mem_f;
                        }
                        if(!isa<llvm::Argument>(val_mem_s))
                        {
                            if(llvm::Instruction *val_mem_s_inst = llvm::dyn_cast<llvm::Instruction>(val_mem_s))
                            {
                                val_s = map_to_cloned_instr.at(val_mem_s_inst);    
                            }
                            func_cont f_obj;
                            f_obj.func = memcpy_func;
                            f_obj.context_arr = src_context;
                            if(FuncToMovedArgMap.at(f_obj).find(val_mem_s) != FuncToMovedArgMap.at(f_obj).end())
                            {
                                val_s = FuncToMovedArgMap.at(f_obj).at(val_mem_s);
                            }
                            // val_s = FuncToVMap.at(target_func).lookup(val_mem_s);
                        }
                        else
                        {
                            // val_mem_s = get_true_value(CI_mem->getArgOperand(1), src_context);
                            function_context f_obj;
                            f_obj.func = memcpy_func;
                            f_obj.context_arr = src_context;

                            if(FuncToMovedArgMap.at(f_obj).find(val_mem_s) != FuncToMovedArgMap.at(f_obj).end())
                            {
                                val_mem_s = FuncToMovedArgMap.at(f_obj).at(val_mem_s);
                            }
                            val_s = val_mem_s;
                        }
                        bitcast_val_f = new llvm::BitCastInst(val_f, llvm::Type::getInt8PtrTy(M.getContext()), "bitcast_mem.gsohc_", target_instr);

                        //Load val_s to a pointer
                        llvm::LoadInst *load_val_s = new llvm::LoadInst(val_s->getType()->getPointerElementType(), val_s, "sec_pointer_load.gsohc", target_instr);
                        load_val_s->moveBefore(target_instr);

                        bitcast_val_s = new llvm::BitCastInst(load_val_s, llvm::Type::getInt8PtrTy(M.getContext()), "bitcast_mem.gsohc_", (llvm::Instruction*)nullptr);
                        bitcast_val_s->insertBefore(target_instr);

                        arg_val.push_back(bitcast_val_f);
                        arg_val.push_back(bitcast_val_s);
                        arg_val.push_back(CI_mem->getArgOperand(2));
                        arg_val.push_back(CI_mem->getArgOperand(3));
                        llvm::CallInst *copy_mem = llvm::CallInst::Create(CI_mem->getCalledFunction(), arg_val, "call_mem.gsohc_", target_instr); 
                        copy_mem->setDebugLoc(target_instr->getDebugLoc());
                    }
                    processed_instr.insert(memcpy_instr);
                }
            }
        }
        // remove the old call instructions
        for(auto iter = old_to_new_callsite.begin(); iter!=old_to_new_callsite.end(); iter++)
        {
            llvm::Instruction *old_call = iter->first;
            old_call->eraseFromParent();
        }

        for(auto iter = processed_instr.begin(); iter!=processed_instr.end(); iter++)
        {
            llvm::Instruction *instr_at_cloned;
            if(map_to_cloned_instr.find(*iter) == map_to_cloned_instr.end())
            {
                continue;
            }
            else
            {
                instr_at_cloned = map_to_cloned_instr.at(*iter);
            }
            instr_at_cloned->eraseFromParent();
        }
        
        //remove the old functions and keep only the cloned functions
        for(auto iter = func_to_cloned_func.begin(); iter!=func_to_cloned_func.end(); iter++)
        {
            llvm::Function *old_func = iter->first;
            llvm::Function *new_func = iter->second;
            old_func->eraseFromParent();
        }
        
        printCustomMessage("Transformation ended successfully!!", "success");
    }
};
//count number of lines in the LLVM Module
struct LineCountPass
{
    int run(llvm::Module &M)
    {
        int line_count = 0;
        for(auto f_iter = M.begin(); f_iter!=M.end(); f_iter++)
        {
            for(auto bb_iter = f_iter->begin(); bb_iter!=f_iter->end(); bb_iter++)
            {
                for(auto instr_iter = bb_iter->begin(); instr_iter!=bb_iter->end(); instr_iter++)
                {
                    line_count++;
                }
            }
        }
        return line_count;
    }
};

struct GSOHC : public ModulePass {

    static char ID;
    GSOHC() : ModulePass(ID) {}

    bool runOnModule(Module &M) override {
        printCustomMessage("Running GSOHC analysis...", "info");
        for(auto &F : M)
        {
            if(llvm::Function *func_ptr = llvm::dyn_cast<llvm::Function>(&F))
            {
                if(func_ptr->isDeclaration() || doNameContain(func_ptr->getName(), "_device_stub"))
                {
                    continue;
                }
                DominatorTree *DT = new DominatorTree(F);//Get the Dominator Tree
                PostDominatorTree *PDT = new PostDominatorTree(F);//Get the Post Dom Tree
                LoopInfo *loopInfo = new LoopInfo(*DT); //Get the Loop Info
                func_to_dom[func_ptr] = DT;
                func_to_pdom[func_ptr] = PDT;
                func_to_loop[func_ptr] = loopInfo;
            }
        }
        for (auto &F : M) //Iterate over the functions of the Module
        {
            if(isNameEqual(F.getName(), "main")){

                auto start_tot = std::chrono::high_resolution_clock::now();
                TerminatorInstWrapperPass *TIP = new TerminatorInstWrapperPass(); // Making an instance for the Terminator Instruction Pass
                TIP->run(*(F.getParent())); //Run the Terminator Instruction Pass (PrePass)

                #if READ_WRITE
                    CustomCallGraphWrapperPass *CWP = new CustomCallGraphWrapperPass(); //Making a Custom Call Graph 
                    if(CWP->run(*(F.getParent()))==false)
                    {
                        printCustomMessage("Error while running Call Graph Wrapper Pass analysis", "error");
                        exit(1);
                    }                
                
                    ReadWriteWrapperPass *RWP = new ReadWriteWrapperPass(); // Making an instance for the Read-Write Wrapper Pass
                    if(RWP->run(*(F.getParent()))==false)
                    {
                        printCustomMessage("Error while running Read-Write analysis", "error");
                        exit(1);
                    }
                #endif
                auto start_an = std::chrono::high_resolution_clock::now();
                //run the skipped set identification on every function

                ModuleWrapperPass *MWP = new ModuleWrapperPass(); // Making an instance for the Module Wrapper Pass
                MWP->func_identification(*(F.getParent())); //Identify the skipped set
                if(MWP->run(*(F.getParent()))==false)
                {
                    printCustomMessage("Error while running module wrapper pass","error");
                    exit(1);
                }
                printCustomMessage("Analysis successfully terminated.", "success");
                #if COMPILER_VERBOSE //Dump the Analysis Result
                    print_result_analysis();
                #endif

                //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: Post Processing Region :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::                
                PostProcessPass *PPP = new PostProcessPass(); // Making an instance for the Post Process Pass
                printCustomMessage("Running post process Pass...", "info");
                PPP->run(); //Run the Post Process Pass
                printCustomMessage("Post processing successfully terminated.", "success");
                auto stop_an = std::chrono::high_resolution_clock::now();
                auto duration_an = std::chrono::duration_cast<std::chrono::milliseconds>(stop_an - start_an);

                auto stop_tot = std::chrono::high_resolution_clock::now();
                auto duration_tot = std::chrono::duration_cast<std::chrono::milliseconds>(stop_tot - start_tot);
                
                //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: Line Count Pass :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                LineCountPass *LCP = new LineCountPass(); // Making an instance for the Line Count Pass
                int line_count = LCP->run(*(F.getParent())); //Run the Line Count Pass
                #if FILE_OUTPUT
		            print_result_output(M, duration_an, line_count, duration_tot);
        	    #endif

                //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: Transformation Pass :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::   
                #if TRANSFORM
                    ChangeIRWrapperPass *CIR = new ChangeIRWrapperPass(); // Making an instance for the Change IR Pass
                    CIR->run(*(F.getParent())); //Run the Change IR Pass
                #endif
            }
        }
        printCustomMessage("GSOHC successfully terminated.", "success");
        return false;
    }
}; // end of struct GSOHC
}  // end of anonymous namespace

char GSOHC::ID = 0;
static RegisterPass<GSOHC> X("gsohc_llvm", "Global Synchronization Optimization in Heterogeneous Computing Pass");
static RegisterStandardPasses Y(
    PassManagerBuilder::EP_ModuleOptimizerEarly,
    [](const PassManagerBuilder &Builder,
       legacy::PassManagerBase &PM) { 
         if (Builder.OptLevel >= 0)
            PM.add(new GSOHC()); 
        });
