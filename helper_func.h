#include "header_files.h"

//defining a struct that will hold the information regarding the context of memcpy and target instructions
typedef struct memcpy_context
{
    std::vector<llvm::Instruction*> context_arr;
    llvm::Instruction *instr;

    bool operator==(const memcpy_context &o) const {
        return context_arr == o.context_arr && instr == o.instr;
    }

    bool operator<(const memcpy_context &o) const {
        if (context_arr != o.context_arr) {
            return context_arr < o.context_arr;
        }
        return instr < o.instr;
    }
}mem_cont;

typedef struct function_context
{
    llvm::Function *func;
    std::vector<llvm::Instruction*> context_arr;
    bool operator==(const function_context &o) const {
        return context_arr == o.context_arr && func == o.func;
    }
    bool operator<(const function_context &o) const {
        if (context_arr != o.context_arr) {
            return context_arr < o.context_arr;
        }
        return func < o.func;
    }
}func_cont;

//Global Variables for Interprocedural Analysis
std::map<llvm::CallInst *, std::set<llvm::Value*>>inter_in_read_set; //Interprocedural Read Set-IN
std::map<llvm::CallInst *, std::set<llvm::Value*>>inter_out_read_set; //Interprocedural Read Set-OUT
std::map<llvm::CallInst *, std::set<llvm::Value*>>inter_in_write_set;//Interprocedural Write Set-IN
std::map<llvm::CallInst *, std::set<llvm::Value*>>inter_out_write_set;//Interprocedural Write Set-OUT
std::map<llvm::CallInst *, std::vector<mem_cont>>inter_in_bar_set;//Interprocedural Barrier Set-IN
std::map<llvm::CallInst *, std::vector<mem_cont>>inter_out_bar_set;//Interprocedural Barrier Set-OUT
std::map<mem_cont, std::vector<mem_cont>> map_to_instr; //Map to Instruction to Instruction
std::set<llvm::BasicBlock*> skipped_set; //Set to store the skipped basic blocks (containing the device function calls)
std::set<llvm::Function*> gpu_containing_func; //Set to store the functions containing the GPU Function Calls
std::map<llvm::Instruction*, llvm::Instruction*> correct_instruction_target; //Contain the correct memcpy to target instruction map (No Jargon)
std::map<llvm::Function*, std::vector<llvm::ReturnInst*>>func_to_return; //Map to store the return instructions of the functions
std::map<llvm::Function*, llvm::DominatorTree*> func_to_dom; //store per function basis dominator tree pass
std::map<llvm::Function*, llvm::PostDominatorTree*> func_to_pdom; //store per function basis post dominator tree pass
std::map<llvm::Function*, llvm::LoopInfo*> func_to_loop; //store per function basis loop info pass 
std::map<llvm::Function*, int>func_to_callsite; //Store per function basis callsite count
std::map<mem_cont, mem_cont> correct_map; //correct target statements
std::map<llvm::Function*, llvm::Function*> func_to_cloned_func; //Map to store the cloned function
std::map<llvm::CallInst*, bool> inter_in_isGPUCall; //Map to store the flag to understand whether GPU Function Calls have been visited
std::map<llvm::CallInst*, bool> inter_out_isGPUCall; 
//Map to store the instruction to context mapping (For those instructions which are same and called from a single function but the callee
// is different)
std::set<std::pair<llvm::Instruction*, std::vector<llvm::Instruction*>>> trans_instruction_mul_context;
//Just to idenitfy whether the function argument moved to the common caller is already moved from another call site
std::set<llvm::Instruction*> moved_argument;

// Define ANSI color code constants
const std::string RESET = "\033[0m";
const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string BLUE = "\033[34m";
const std::string CYAN = "\033[36m";
const std::string BOLD = "\033[1m";


//defining the read/write status of the param of the function
typedef struct rw_status
{
    bool read =true, write = false;
}rw_stat;

#define PRINT_FUNCTION_STACK 0

#define PRINT_READ_WRITE_STAT 0

#define DEBUG 0

#define POST_PROCESS_MAP 1

using namespace llvm;

std::map<llvm::Function*, std::vector<rw_stat>>param_rw_stat; //Map to collect the information about the read/write status of each argument of Functions.

//------------------------------------------------------------------------ DECLARATION -------------------------------------------------------------------
//Make the function stack for Read-Write Pass
void make_stack(std::vector<llvm::Function*>&function_stack);

//Check if the instruction is a GPU Function Call
bool isGPUFunctionCall(llvm::Instruction *instr);

void find_rw(llvm::Function *func, std::map<llvm::Function*, std::vector<rw_stat>>&param_rw_stat,
                std::map<llvm::Function*, std::vector<llvm::Value*>>&func_param, Module &M);

void make_rw_analysis(std::map<llvm::Function*, std::vector<rw_stat>>&param_rw_stat,
                        std::vector<llvm::Function*>&function_stack, Module &M);

//Return True LLVM::Value from a Intermediate Value for a Function
llvm::Value* get_true_value(llvm::Value *val);

//Return True LLVM::Value from a Intermediate Value for a Function (For Alias Purposes)
llvm::Value* get_true_value(llvm::Value *val, std::vector<llvm::Instruction*>caller_instr_stack);

//Check whether both names are equal or not
bool isNameEqual(llvm::StringRef name, std::string str);

//Get the name of the Function called at the Call Instruction
llvm::StringRef getName(llvm::CallInst* CI);

//Check if the API Function name contains the given string or not
bool doNameContain(llvm::StringRef api_name, std::string str);

//Return Actual Device Function Name
std::string get_device_func_name(std::string input_string);

//return the index from the given variable, returns -1 if not found
int RetIndexArg(llvm::Value *val, llvm::Instruction *instr);

//Return bool whether the argument is being read or being writtehn of a given function
bool isBeingRead(llvm::CallInst *CI, std::map<llvm::Function*, std::vector<rw_stat>>rw_stat_map, int index, bool isGPUCall);

//Insert into Local Reads or Local Write Set
void insert_local_rw(llvm::Value *val_r, llvm::Value *val_w, std::set<llvm::Value*>&local_read, std::set<llvm::Value*>&local_write, 
            std::vector<llvm::Instruction*> instr_stack);

//Generate the Mapping of Memcpy Statements
void generate_memcpy_map(std::vector<mem_cont> in_bar, bool &kill, llvm::Instruction *curr_instr, std::vector<llvm::Instruction *>instr_stack);

//Generate the Mapping Key States
void generate_sets(llvm::Instruction *curr_instr, std::map<llvm::Instruction*, std::set<llvm::Value*>> &read_set,
    std::map<llvm::Instruction*, std::set<llvm::Value*>> &write_set, std::map<llvm::Instruction*, std::vector<llvm::Instruction*>> &bar_set);

//unify two sets and put the result in the resultant set
void unify_set(std::set<llvm::Value*> &result, std::set<llvm::Value*> input_1, std::set<llvm::Value*> input_2);

//unify two sets and put the result in the resultant set
void unify_vector(std::vector<llvm::Instruction*> &result, std::vector<llvm::Instruction*> input_1, std::vector<llvm::Instruction*> input_2);

//Return the successor instructions of this instruction
std::vector<llvm::Instruction*> getSuccessorInstructions(llvm::Instruction *instr);

//Return true if the sets has changed
bool set_changed(std::set<llvm::Value*>set_1, std::set<llvm::Value*>set_2);

//Get the predecessor Instructions of a given instruction
std::vector<llvm::Instruction*> getPredecessorInstructions(llvm::Instruction *instr);

//Check if the Function Arguments are data flow facts or not
bool isArgFact(llvm::CallInst *CI, std::set<llvm::Value*>read_set, std::set<llvm::Value*>write_set, std::vector<llvm::Instruction*>instr_stack);

//Print a set for Debugging
void print_set(std::set<llvm::Value*>set);

//Check if the two vectors are same or not 
bool isSameVector(std::vector<llvm::Instruction*>vec_1, std::vector<llvm::Instruction*>vec_2);

//check if the current instruction resides in the barrier set
bool isInstrContainInBarrier(llvm::Instruction *instr, std::vector<llvm::Instruction*> instr_stack,
    mem_cont &obj);

//Print the result of Analysis (map_to_instr)
void print_result_analysis();

//find the intersection of given two sets and store it in the later
void intersect_set(std::set<llvm::Value*>&set1, std::set<llvm::Value*> set2);

//Find the intersection between two vectors
void intersect_vector(std::vector<mem_cont>vec_1, std::vector<mem_cont>vec_2, std::vector<mem_cont> &result);

//Return the Common Caller Index of the COntextual Path
int getCommonCallerIndex(std::vector<llvm::Instruction*>context_stack_1, std::vector<llvm::Instruction*>context_stack_2);

//Check if the Load Instruction is passed as an argument to a corresponding function
bool isLoadPassedArg(llvm::LoadInst *LI);

//Return the function where memcpy and target functions are intersecting and they also are called from different call sites
llvm::Instruction *memcpy_target_intersect(std::vector<llvm::Instruction*> memcpy_stack, std::vector<llvm::Instruction*> target_stack);


//------------------------------------------------------------------------ DEFINITION -------------------------------------------------------------------

llvm::Instruction *memcpy_target_intersect(std::vector<llvm::Instruction*> memcpy_stack, std::vector<llvm::Instruction*> target_stack)
{
    int min_size = std::min(memcpy_stack.size(), target_stack.size());
    for(auto i = 0; i<min_size; i++)
    {
        // Get the functions containing these instructions
        llvm::Function *memcpy_func, *target_func;
        if(llvm::CallInst *memcpy_call = llvm::dyn_cast<llvm::CallInst>(memcpy_stack[i]))
        {
            memcpy_func = memcpy_call->getCalledFunction();
        }
        if(llvm::CallInst *target_call = llvm::dyn_cast<llvm::CallInst>(target_stack[i]))
        {
            target_func = target_call->getCalledFunction();
        }

        // If functions match but call sites are different, we found our intersection point
        if (memcpy_func == target_func && memcpy_stack[i] != target_stack[i]) {
            return target_stack[i];
        }
    }
    return nullptr;// No such function found where memcpy and target functions are intersecting and they also are called from different call sites
}

bool isLoadPassedArg(llvm::LoadInst *LI)
{
    //is the current Load Instruction passed as an argument to a function
    //Use Def-Use Chain
    if(LI->getType()->isPointerTy())
    {
        return true;
    }
    return false;
}

void intersect_vector(std::vector<mem_cont>vec_1, std::vector<mem_cont>vec_2, std::vector<mem_cont> &result)
{
    for(auto iter=vec_1.begin(); iter!=vec_1.end(); iter++)
    {
        if(std::find(vec_2.begin(), vec_2.end(), *iter)!=vec_2.end())
        {
            result.push_back(*iter);
        }
    }
}

void intersect_set(std::set<llvm::Value*>&set1, std::set<llvm::Value*> set2)
{
    std::set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), std::inserter(set1, set1.begin()));
}

//Can be used for debug purposes
void print_result_analysis()
{
    for(auto iter=map_to_instr.begin(); iter!=map_to_instr.end(); iter++)
    {
        errs()<<"Target Location: "<<iter->first.instr->getDebugLoc().getLine()<<"---->";
        for(auto iter_j = iter->second.begin(); iter_j!=iter->second.end(); iter_j++)
        {
            errs()<<"Memcpy Location: "<<iter_j->instr->getDebugLoc().getLine()<<", ";
            //print context
            std::vector<llvm::Instruction*> context_arr = iter_j->context_arr;
            for(auto iter_k = context_arr.begin(); iter_k!=context_arr.end(); iter_k++)
            {
                llvm::errs()<<"Context: "<<(*iter_k)->getDebugLoc().getLine()<<" ";
            }
        }
        errs()<<"\n";
    }
}

bool isInstrContainInBarrier(llvm::Instruction *instr, std::vector<llvm::Instruction*> instr_stack, 
    mem_cont &obj)
{
    for(auto iterator = map_to_instr.begin(); iterator!=map_to_instr.end(); iterator++)
    {
        if(iterator->first.instr == instr && iterator->first.context_arr == instr_stack)
        {
            obj = iterator->first;
            return true;
        }
    }
    return false;
        
}

bool isSameVector(std::vector<llvm::Instruction*>vec_1, std::vector<llvm::Instruction*>vec_2)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: isSameVector\n";
    #endif
    if(vec_1.size() != vec_2.size())
    {
        return false;
    }
    for(int i=0; i<vec_1.size(); i++)
    {
        if(vec_1[i] != vec_2[i])
        {
            return false;
        }
    }
    return true;
}

void print_set(std::set<llvm::Value*>set)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: print_set\n";
    #endif
    for(auto iterator = set.begin(); iterator!=set.end(); iterator++)
    {
        errs()<<**iterator<<" ";
    }
    errs()<<"\n";
}

bool isArgFact(llvm::CallInst *CI, std::set<llvm::Value*>read_set, std::set<llvm::Value*>write_set, std::vector<llvm::Instruction*>instr_stack)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: isArgFact\n";
    #endif
    for(auto args = CI->arg_begin(); args!=CI->arg_end(); args++)
    {
        if(llvm::Value *arg = llvm::dyn_cast<llvm::Value>(args))
        {
            if(read_set.find(get_true_value(arg, instr_stack))!=read_set.end() || write_set.find(get_true_value(arg, instr_stack))!=write_set.end())
            {
                return true;
            }
        }
    }
    return false;
        
}

std::vector<llvm::Instruction*> getPredecessorInstructions(llvm::Instruction *instr)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: getPredecessorInstructions\n";
    #endif
    std::vector<llvm::Instruction*>predecessor_instr;
    llvm::BasicBlock *contain_bb = instr->getParent();
    llvm::Instruction *first_instr = &contain_bb->front();
    if(first_instr!=instr)
    {
        std::vector<llvm::Instruction*> temp_vector;
        for(auto iterator = contain_bb->begin(); iterator!=contain_bb->end(); iterator++)
        {
            if(&*iterator == instr)
            {
                break;
            }
            temp_vector.push_back(&*iterator);
        }
        predecessor_instr.push_back(temp_vector.back());
    }
    else
    {
        for(auto pred_bb = pred_begin(contain_bb); pred_bb!=pred_end(contain_bb); pred_bb++)
        {
            predecessor_instr.push_back(&(*pred_bb)->back());
        }
    }
    return predecessor_instr;
}

bool set_changed(std::set<llvm::Value*>set_1, std::set<llvm::Value*>set_2)
{
    if(set_1.size() != set_2.size())
    {
        return true;
    }
    for(auto iterator = set_1.begin(); iterator!=set_1.end(); iterator++)
    {
        if(set_2.find(*iterator) == set_2.end())
        {
            return true;
        }
    }
    return false;
}

std::vector<llvm::Instruction*> getSuccessorInstructions(llvm::Instruction *instr)
{
    std::vector<llvm::Instruction*>successor_instr;
    // errs()<<*instr<<"\n"; //debug
    if(!instr->isTerminator())
    {
        successor_instr.push_back(instr->getNextNode());
    }
    else
    {
        for(int i=0; i<instr->getNumSuccessors(); i++)
        {
            llvm::BasicBlock *succ_bb = instr->getSuccessor(i);
            successor_instr.push_back(&succ_bb->front());
        }
    }
    return successor_instr;
}

void unify_vector(std::vector<mem_cont> &result, std::vector<mem_cont> input_1, std::vector<mem_cont> input_2)
{
    for(auto iterator = input_1.begin(); iterator!=input_1.end(); iterator++)
    {
        if(std::find(result.begin(), result.end(), *iterator) == result.end())
        {
            result.push_back(*iterator);
        }
    }
    for(auto iterator = input_2.begin(); iterator!=input_2.end(); iterator++)
    {
        if(std::find(result.begin(), result.end(), *iterator) == result.end())
        {
            result.push_back(*iterator);
        }
    }

}

void unify_set(std::set<llvm::Value*> &result, std::set<llvm::Value*> input_1, std::set<llvm::Value*> input_2)
{
    std::set_union(input_1.begin(), input_1.end(), input_2.begin(), input_2.end(), std::inserter(result, result.begin()));
}

void generate_sets(llvm::Instruction *curr_instr, std::map<llvm::Instruction*, std::set<llvm::Value*>> &read_set,
    std::map<llvm::Instruction*, std::set<llvm::Value*>> &write_set, std::map<llvm::Instruction*, std::vector<mem_cont>> &bar_set, 
    std::vector<llvm::Instruction *>instr_stack)
{
    if(read_set.find(curr_instr) == read_set.end())
    {
        read_set.insert(std::pair<llvm::Instruction*, std::set<llvm::Value*>>(curr_instr, {}));
    }
    if(write_set.find(curr_instr) == write_set.end())
    {
        write_set.insert(std::pair<llvm::Instruction*, std::set<llvm::Value*>>(curr_instr, {}));
    }
    if(bar_set.find(curr_instr) == bar_set.end())
    {
        bar_set.insert(std::pair<llvm::Instruction*, std::vector<mem_cont>>(curr_instr, {}));
    }
}


void generate_memcpy_map(std::vector<mem_cont> in_bar, bool &kill, llvm::Instruction *curr_instr, std::vector<llvm::Instruction *>instr_stack)
{   
    kill=true;
    mem_cont obj = mem_cont();
    if(isInstrContainInBarrier(curr_instr, instr_stack, obj))
    {
        for(auto iterator = in_bar.begin(); iterator != in_bar.end(); iterator++)
        {
            map_to_instr[obj].push_back(*iterator);
        }
    } 
    else
    {
        obj.instr = curr_instr;
        obj.context_arr = instr_stack;
        map_to_instr.insert(std::pair<mem_cont, std::vector<mem_cont>>(obj, in_bar));
    }
}


void insert_local_rw(llvm::Value *val_r, llvm::Value *val_w, std::set<llvm::Value*>&local_read, std::set<llvm::Value*>&local_write,
                std::vector<llvm::Instruction*> instr_stack)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: insert_local_rw\n";
    #endif
    local_read.insert(get_true_value(val_r, instr_stack));
    local_write.insert(get_true_value(val_w, instr_stack));
}
bool isBeingRead(llvm::CallInst *CI, std::map<llvm::Function*, std::vector<rw_stat>>rw_stat_map, int index, bool isGPUCall)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: isBeingRead\n";
    #endif
    llvm::Function *called_func = CI->getCalledFunction();
    if(isGPUCall)
    {
        llvm::Function *kernel_func = CI->getModule()->getFunction(get_device_func_name(called_func->getName().str()));
        if(rw_stat_map.at(kernel_func)[index].read)
        {
            return true;
        }
        else{
            return false;
        }
    }
    else{
        if(rw_stat_map.at(called_func)[index].read)
        {
            return true;
        }
        else{
            return false;
        }
    }
}


//Return Actual Device Function Name
std::string get_device_func_name(std::string input_string)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: get_device_func_name\n";
    #endif
    int function_name_length = stoi(input_string.substr(2,2));
    int act_func_name_length = function_name_length - 15; // Remove the String Length for "__device_stub__"
    std::string actual_function_name = input_string.substr(19);
    actual_function_name = "_Z" + std::to_string(act_func_name_length) + actual_function_name;
    return actual_function_name;
}

int RetIndexArg(llvm::Value *val, llvm::CallInst *CI)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: RetIndexArg\n";
    #endif
    llvm::Function *called_function = CI->getCalledFunction();
    int counter = 0; //initializing the index to zero
    for(auto arg=called_function->arg_begin(); arg!=called_function->arg_end(); arg++)
    {
        if(llvm::Value *arg_val = llvm::dyn_cast<llvm::Value>(arg))
        {
            if(arg_val == val)
            {
                return counter;
            }
        }
        counter++;
    }
    return -1;
}

//Get the name of the Function called at the Call Instruction
llvm::StringRef getName(llvm::CallInst* CI)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: getName\n";
    #endif
    llvm::Function* calledF = CI->getCalledFunction();
    if(calledF)
    {
        return calledF->getName();
    }
    else
    {
        llvm::errs()<<"--------------------------------------ERROR: Function Name could not be retrieved!!--------------------------------------------\n";
        llvm::errs()<<"Inside Function: helper_function/getName()\n";
        exit(1);
    }
}

bool isGPUFunctionCall(llvm::Instruction *instr)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: isGPUFunctionCall\n";
    #endif
    if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(instr))
    {
        llvm::Function *called_func = CI->getCalledFunction();
        if(called_func == nullptr)
        {
            return false;
        }
        if(doNameContain(CI->getCalledFunction()->getName(), "_device_stub_"))
        {
            return true;
        }else
            return false;
    }
}

//Check if the API Function name contains the given string or not
bool doNameContain(llvm::StringRef api_name, std::string str)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: doNameContain\n";
    #endif
    if(api_name.contains(str))
    {
        return true;
    }
    else
    {
        return false;
    }
}

//Check whether both names are equal or not
bool isNameEqual(llvm::StringRef name, std::string str)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: isNameEqual\n";
    #endif
    if(name.equals(str))
    {
        return true;
    }
    else
    {
        return false;
    }
}

llvm::Value* get_true_value(llvm::Value *val)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: get_true_value\n";
    #endif
    std::vector<llvm::Instruction*>instr_set = {}; // Instruction Set to iterate over
    if(llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(val))
    {
        instr_set.push_back(I); // Insert the instruction into the instr_set to recursively travese
    }
    while(!instr_set.empty())
    {
        auto iter_instr = instr_set.begin();
        // llvm::errs()<<**iter_instr<<" ";
        if(llvm::isa<llvm::AllocaInst>(*iter_instr))
        {
            return *iter_instr;
        }
        else if (llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(*iter_instr))
        {
            return *iter_instr;
        }
        else if(llvm::LoadInst *LI = llvm::dyn_cast<llvm::LoadInst>(*iter_instr))
        {
            // return LI->getPointerOperand();
            llvm::Value *val_inserter = LI->getPointerOperand();
            if(llvm::isa<llvm::GlobalValue>(val_inserter))
            {
                return val_inserter;
            }
            if(llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(val_inserter))
            {
                instr_set.push_back(I);
            }
            else if(llvm::Argument *arg = llvm::dyn_cast<llvm::Argument>(val_inserter))
            {
                return val_inserter;
            }
        }
        else if(llvm::GetElementPtrInst *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(*iter_instr))
        {
            llvm::Value *val_inserter = GEP->getPointerOperand();

            if(llvm::isa<llvm::GlobalValue>(val_inserter))
            {
                return val_inserter;
            }
            if(llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(val_inserter))
            {
                instr_set.push_back(I);
            }
            else if(llvm::Argument *arg = llvm::dyn_cast<llvm::Argument>(val_inserter))
            {
                return val_inserter;
            }
            // return GEP->getPointerOperand();
        }
        else if(llvm::CastInst *CSI = llvm::dyn_cast<llvm::CastInst>(*iter_instr))
        {
            llvm::Value *val_inserter = CSI->getOperand(0);
            // llvm::errs()<<"----->"<<*val_inserter;
            if(llvm::isa<llvm::GlobalValue>(val_inserter))
            {
                return val_inserter;
            }
            if(llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(val_inserter))
            {
                instr_set.push_back(I);
            }
            else if(llvm::Argument *arg = llvm::dyn_cast<llvm::Argument>(val_inserter))
            {
                return val_inserter;
            }
        }
        instr_set.erase(instr_set.begin());
    }
}

//Return True LLVM::Value from a Intermediate Value for a Function
llvm::Value* get_true_value(llvm::Value *val, std::vector<llvm::Instruction*>caller_instr_stack)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: get_true_value\n";
    #endif
    if(caller_instr_stack.empty())
    {
        return get_true_value(val);
    }
    std::vector<llvm::Value*>instr_set = {}; // Instruction Set to iterate over
    instr_set.push_back(val); // Push the value in the instruction set
    while(!instr_set.empty())
    {
        auto iter_instr = instr_set.begin();
        if(llvm::isa<llvm::AllocaInst>(*iter_instr))
        {
            return *iter_instr;
        }
        else if(llvm::Argument *arg = llvm::dyn_cast<llvm::Argument>(*iter_instr))
        {
            llvm::Instruction *caller_instr = caller_instr_stack.back();
            if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(caller_instr))
            {
                llvm::Function *caller_func = CI->getCalledFunction();
                int iter_count = 0; //takes the count of the argument index
                for(auto arg_iter = caller_func->arg_begin(); arg_iter!=caller_func->arg_end(); arg_iter++)
                {
                    if(llvm::Argument* val_arg_iter = llvm::dyn_cast<llvm::Argument>(arg_iter))
                    {
                        if(val_arg_iter == arg)
                        {
                            break;
                        }
                    }
                    iter_count++;
                }
                //get the argument at that particular index
                llvm::Value *val_inserter = CI->getArgOperand(iter_count);
                instr_set.push_back(val_inserter);
            }
            else
            {
                errs()<<"Error: The caller instruction is not a Call Instruction\n";
                exit(1);
            }
            caller_instr_stack.pop_back();
            
        }
        else if (llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(*iter_instr))
        {
            return *iter_instr;
        }
        else if(llvm::LoadInst *LI = llvm::dyn_cast<llvm::LoadInst>(*iter_instr))
        {
            // return LI->getPointerOperand();
            llvm::Value *val_inserter = LI->getPointerOperand();
            if(llvm::isa<llvm::GlobalValue>(val_inserter))
            {
                return val_inserter;
            }
            if(llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(val_inserter))
            {
                instr_set.push_back(val_inserter);
            }
            else if(llvm::Argument *arg = llvm::dyn_cast<llvm::Argument>(val_inserter))
            {
                instr_set.push_back(val_inserter);
            }
        }
        else if(llvm::GetElementPtrInst *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(*iter_instr))
        {
            llvm::Value *val_inserter = GEP->getPointerOperand();

            if(llvm::isa<llvm::GlobalValue>(val_inserter))
            {
                return val_inserter;
            }
            if(llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(val_inserter))
            {
                instr_set.push_back(val_inserter);
            }
            else if(llvm::Argument *arg = llvm::dyn_cast<llvm::Argument>(val_inserter))
            {
                instr_set.push_back(val_inserter);
            }
            // return GEP->getPointerOperand();
        }
        else if(llvm::CastInst *CSI = llvm::dyn_cast<llvm::CastInst>(*iter_instr))
        {
            llvm::Value *val_inserter = CSI->getOperand(0);
            // llvm::errs()<<"----->"<<*val_inserter;
            if(llvm::isa<llvm::GlobalValue>(val_inserter))
            {
                return val_inserter;
            }
            if(llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(val_inserter))
            {
                instr_set.push_back(val_inserter);
            }
            else if(llvm::Argument *arg = llvm::dyn_cast<llvm::Argument>(val_inserter))
            {
                instr_set.push_back(val_inserter);;
            }
        }
        instr_set.erase(instr_set.begin());
    }
}


//Make the function stack for Read-Write Pass
void make_stack(std::vector<llvm::Function*>&function_stack,
    std::map<llvm::Function*, std::set<llvm::Function*>>global_callgraph, Module &M)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: make_stack\n";
    #endif
    std::set<llvm::Function*> main_callee_set = global_callgraph.at(M.getFunction("main")); // get the callee functions of function main
    function_stack.push_back(M.getFunction("main")); // push the main function in the function stack

    std::set<llvm::Function*>visited; //Mark Visited for Functions 

    std::vector<llvm::Function*>worklist_func; //worklist for Functions 
    worklist_func.push_back(M.getFunction("main"));


    while(!worklist_func.empty())
    {
        llvm::Function *worklist_front_function = *(worklist_func.begin()); //get the first element from the worklist
        visited.insert(worklist_front_function); //mark the front function as visited

        std::set<llvm::Function*> worklist_front_callee_set = global_callgraph.at(worklist_front_function); //copy the calllee functions of the front function in LLVM

        for(auto iterator = worklist_front_callee_set.begin(); iterator!=worklist_front_callee_set.end(); iterator++)
        {
            
            if(visited.find(*iterator)==visited.end())
            {
                worklist_func.push_back(*iterator); //push each callee in the worklist stack
                function_stack.push_back(*iterator); //push back in the function stack
            }
        }

        worklist_func.erase(worklist_func.begin());
    }

    #if PRINT_FUNCTION_STACK
            errs()<<"<------------------------------------------------------------------------ Printing the Function Stack -------------------------------------------------------------->"<<"\n";
            for(auto iterator = function_stack.begin(); iterator!=function_stack.end(); iterator++)
            {
                errs()<<(*iterator)->getName().str()<<"\n";
            }
    #endif
}


void find_rw(llvm::Function *func, std::map<llvm::Function*, std::vector<rw_stat>>&param_rw_stat,
                std::map<llvm::Function*, std::vector<llvm::Value*>>&func_param, Module &M)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: find_rw\n";
    #endif
    //Iterate over the Function Basic Block by Basic Block
    int index = 0;
    for(auto iter = func_param.at(func).begin(); iter!=func_param.at(func).end(); iter++)
    {
        for(auto bb_iter=func->begin(); bb_iter!=func->end();bb_iter++)
        {
            for(auto inst_iter = bb_iter->begin(); inst_iter!=bb_iter->end(); inst_iter++)
            {
                //If the Instruction is a Store Instruction
                if(llvm::StoreInst *store = llvm::dyn_cast<llvm::StoreInst>(inst_iter))
                {
                    llvm::Value *written_val = store->getPointerOperand();
                    llvm::Value *written_true_val = get_true_value(written_val); // Get the True Written True Value

                    // errs()<<*written_true_val<<" ------------------------------ "<<**iter<<"\n";

                    if(*iter == written_true_val)
                    {
                        rw_stat obj; // have an object of the structure
                        //Marking as Written Value
                        obj.read = false;
                        obj.write = true;
                        param_rw_stat.at(func)[index] = obj; // Mark as Written
                        break;
                    }
                }
                //If the Instruction is a Call Instruction
                else if(llvm::CallInst *call_inst = llvm::dyn_cast<llvm::CallInst>(inst_iter))
                {
                    llvm::Function *called_f = call_inst->getCalledFunction();
                    //Check If the function is not recursively calling itself
                    if(called_f!=func)
                    {
                        //Check If the Function has been defined inside the translation unit or not
                        if(called_f!=nullptr)
                        {
                            if(!(called_f->isDeclaration()))
                            {
                                if(param_rw_stat.find(called_f)==param_rw_stat.end())
                                {
                                    int index;
                                    for(auto param_iter=called_f->arg_begin();param_iter!=called_f->arg_end();param_iter++)
                                    {
                                        rw_stat obj;
                                        obj.read = false;
                                        obj.write = true;
                                        param_rw_stat[called_f].push_back(obj);
                                    }
                                }
                                int callee_index = 0; // Try to Find the index at which the param got matched
                                for(auto param_iter=called_f->arg_begin();param_iter!=called_f->arg_end();param_iter++)
                                {
                                    llvm::Value *true_arg = call_inst->getArgOperand(callee_index);

                                    // llvm::Value *true_arg = get_true_value(arg);
                                    //Check For Aliasness
                                    if(*iter == true_arg)
                                    {
                                        if(!doNameContain(called_f->getName(), "device_stub"))
                                        {
                                            param_rw_stat.at(func)[index] = param_rw_stat.at(called_f)[callee_index]; // Get the Value is being written or not
                                        }
                                        if(doNameContain(called_f->getName(), "device_stub"))
                                        {
                                            // llvm::Function *kernel_func = M.getFunction(get_device_func_name(called_f->getName().str()));
                                            for(auto param_iter=param_rw_stat.begin();param_iter!=param_rw_stat.end();param_iter++)
                                            {
                                                llvm::Function *kernel_func = param_iter->first;
                                                if(isNameEqual(kernel_func->getName(), get_device_func_name(called_f->getName().str())))
                                                {
                                                    param_rw_stat.at(func)[index] = param_rw_stat.at(kernel_func)[callee_index]; // Get the Value is being written or not 
                                                }
                                            }
                                            // llvm::errs()<<"Kernel Function Name: "<<kernel_func->getName().str()<<"\n";
                                            // param_rw_stat.at(func)[index] = param_rw_stat.at(kernel_func)[callee_index]; // Get the Value is being written or not 
                                        }
                                    }
                                    callee_index++; //Increment index to get the parameter index from the function
                                }
                            }
                        }
                    }
                    
                }
            }
            //Check whether the value has been found to be written 
            if(param_rw_stat.at(func)[index].write)
            {
                break;
            }
        }
        index++;
    }
}

void make_rw_analysis(std::map<llvm::Function*, std::vector<rw_stat>>&param_rw_stat,
                        std::vector<llvm::Function*>&function_stack, Module &M)
{
    #if DEBUG
        errs()<<"DEBUGGING: Function Name: make_rw_analysis\n";
    #endif
    std::map<llvm::Function*, std::vector<llvm::Value*>>func_param;// store the function parameters w.r.t to each function

    //Do the analysis for left over functions (important for kernel calls)
    for(auto iterator = M.begin(); iterator!=M.end(); iterator++)
    {
        if (llvm::Function *module_func = llvm::dyn_cast<llvm::Function>(iterator))
        {
            // llvm::errs()<<"Function Name: "<<module_func->getName().str()<<"\n";
            if(std::find(function_stack.begin(), function_stack.end(), module_func)==function_stack.end())
            {
                if(doNameContain(module_func->getName(), "device_stub"))
                {
                    llvm::Function *kernel_func = M.getFunction(get_device_func_name(module_func->getName().str()));
                    if(kernel_func !=nullptr)
                    {
                        function_stack.push_back(kernel_func);
                    }
                }
                else
                {
                    function_stack.push_back(module_func);
                }
            }
        }
    }
    while(!function_stack.empty())
    {
        llvm::Function *top_function = function_stack.back();

        func_param.insert(std::pair<llvm::Function*, std::vector<llvm::Value*>>(top_function,{}));
        param_rw_stat.insert(std::pair<llvm::Function*, std::vector<rw_stat>>(top_function, {}));
        
        //Iterate over function arguments
        for (auto &arg : top_function->args())
        {
            if(llvm::Value* par = llvm::dyn_cast<llvm::Value>(&arg))
            {
                //Insert in the order to the Function Map
                func_param.at(top_function).push_back(par);
            }
        }
        for(auto iter = func_param.at(top_function).begin();iter!=func_param.at(top_function).end();iter++)
        {
            rw_stat obj; //Default Object of the Struct as Saying Read as true and other as False
            param_rw_stat.at(top_function).push_back(obj); //Mark the params as undefined
        }

        find_rw(top_function, param_rw_stat, func_param, M); //Find the read-write set for the function parameters
        function_stack.pop_back();
    }

    #if PRINT_READ_WRITE_STAT

        for(auto iterator = param_rw_stat.begin(); iterator!=param_rw_stat.end(); iterator++)
        {
            errs()<< iterator->first->getName().str()<<"---------------\n";
            for(auto iterator_j = iterator->second.begin(); iterator_j != iterator->second.end(); iterator_j++)
            {
                errs()<<"Read: "<<iterator_j->read<<" Write: "<<iterator_j->write<<" ++++++ ";
            }
            errs()<<"\n";
        }
    #endif
}

//------------------------------------------------------------------------------------------- FUNCTIONS FOR POST-PROCESSING STEP ----------------------------------------------------------------------------------------------------
//copy the key of map_to_instr to another map as values and values as keys
void post_process_copy(std::map<mem_cont, std::vector<mem_cont>>&opp_copy);

//Find for equivalent entry in mem_cont vector
bool isSameEntry(std::vector<mem_cont>vec_1, mem_cont obj);

// Check for occurance in same loop or PDT and DT 
bool isSameLoopOrPDT(llvm::Instruction *memcpy_instr, llvm::Instruction *target_instr, 
    llvm::DominatorTree *dom, llvm::PostDominatorTree *pdom, llvm::LoopInfo* loopInfo);

//bool get common Caller Function of two context stacks
llvm::Function * getCommonCaller(std::vector<llvm::Instruction*>context_stack_1, std::vector<llvm::Instruction*>context_stack_2);

//Get the number of call sites correponding to each function in a trasnlation unit
void getNumCallSite(Module *M, std::map<llvm::Function*, int>call_site_map);

//Get residue context from the contexts
std::vector<llvm::Instruction*> getResidue(std::vector<llvm::Instruction*>context, llvm::Function *common_caller);

//Find the destinatipon function of the source instruction, needed to be interprocedural analysis
llvm::Function * findDestFunction(llvm::Instruction *src, llvm::Instruction *dest, std::vector<llvm::Instruction*>src_context, 
    std::vector<llvm::Instruction*>dest_context, bool &isSourceLevel, llvm::Instruction *&return_site);

bool findDominanceOrder(llvm::Instruction *instr, std::vector<llvm::Instruction*>instr_context, llvm::Instruction *instr2, 
    std::vector<llvm::Instruction*>instr2_context, llvm::DominatorTree *dom);

//get context till destinatiuon function
std::vector<llvm::Instruction*> getContextDest(std::vector<llvm::Instruction*> context, llvm::Function *dest_func);

//Find which dominates which one in the same block (wherher i dominates j)
bool isInSuccSBB(llvm::Instruction *i, llvm::Instruction *j);

//print Output after the whole analysis has terminated
void print_result_output(Module &M);

//Check if all the elements of the vector are same or not
bool ifAllSame(std::vector<mem_cont>vec);

//Debug Function -- Print the contents of a mem_cont obj
void print_mem_cont(mem_cont obj, bool print_context);

// -----------------------------------------------------------------------------------------Definitions----------------------------------------------------------------------------------------

void print_mem_cont(mem_cont obj, bool print_context)
{
    llvm::errs()<<"Mem Cont: "<<obj.instr->getDebugLoc()->getLine()<<"\n";
    if(print_context)
    {
        llvm::errs()<<"Context: ";
        for(auto iterator = obj.context_arr.begin(); iterator!=obj.context_arr.end(); iterator++)
        {
            llvm::errs()<<(*iterator)->getDebugLoc()->getLine()<<" ";
        }
        llvm::errs()<<"\n";
    }
}

bool ifAllSame(std::vector<mem_cont>vec)
{
    for(auto iterator = vec.begin(); iterator!=vec.end(); iterator++)
    {
        if(iterator->instr != vec[0].instr)
        {
            return false;
        }
    }
    return true;
}

std::vector<llvm::Instruction*> getContextDest(std::vector<llvm::Instruction*> context, llvm::Function *dest_func)
{
    std::vector<llvm::Instruction*> return_vector;
    for(auto iter = context.begin(); iter!=context.end(); iter++)
    {
        if((*iter)->getFunction() == dest_func)
        {
            return_vector.push_back(*iter);
            break;
        }
        return_vector.push_back(*iter);
    }
    return return_vector;
}


//Find dominance relationship between two target instruction
bool findDominanceOrder(llvm::Instruction *instr, std::vector<llvm::Instruction*>instr_context, llvm::Instruction *instr2, 
    std::vector<llvm::Instruction*>instr2_context, llvm::DominatorTree *dom)
{
    llvm::Instruction *call_context_1;
    llvm::Instruction *call_context_2;
    int min_size = std::min(instr_context.size(), instr2_context.size());
    for(int i = 0; i<min_size; i++)
    {
        if(instr_context[i]->getFunction() ==instr2_context[i]->getFunction())
        {
            call_context_1 = instr_context[i];
            call_context_2 = instr2_context[i];
        }
    }
    if(dom->dominates(call_context_1->getParent(), call_context_2->getParent()))
    {
        return true;
    }
    return false;
}

void getNumCallSite(Module *M, std::map<llvm::Function*, int>call_site_map)
{
    for(auto iter_m = M->begin(); iter_m!=M->end(); iter_m++)
    {
        if(llvm::Function *func = llvm::dyn_cast<llvm::Function>(iter_m))
        {
            for(auto bb_iter = func->begin(); bb_iter!=func->end(); bb_iter++)
            {
                for(auto inst_iter = bb_iter->begin(); inst_iter!=bb_iter->end(); inst_iter++)
                {
                    if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(inst_iter))
                    {
                        llvm::Function *called_func = CI->getCalledFunction();
                        call_site_map[called_func]++;
                    }
                }
            }
        }

    }
}

int getCommonCallerIndex(std::vector<llvm::Instruction*>context_stack_1, std::vector<llvm::Instruction*>context_stack_2)
{
    int min_size = std::min(context_stack_1.size(), context_stack_2.size());
    int com_i = 0;
    for(int i=0; i<min_size; i++)
    {
        if(context_stack_1[i]->getFunction() == context_stack_2[i]->getFunction())
        {
            com_i = i;
        }
    }
    return com_i;
}

llvm::Function * getCommonCaller(std::vector<llvm::Instruction*>context_stack_1, std::vector<llvm::Instruction*>context_stack_2)
{
    int min_size = std::min(context_stack_1.size(), context_stack_2.size()); //get the max size of two contexts
    if(min_size == 0)
    {
        return nullptr;
    }
    llvm::Function *common_caller;
    for(int i=0; i<min_size; i++)
    {
        llvm::Function *f_1 = context_stack_1[i]->getFunction();
        llvm::Function *f_2 = context_stack_2[i]->getFunction();
        if(f_1 == f_2)
        {
            common_caller = f_1;
        }
    }
    return common_caller;
}

bool isSameLoopOrPDT(llvm::Instruction *memcpy_instr, llvm::Instruction *target_instr, 
    llvm::DominatorTree *dom, llvm::PostDominatorTree *pdom, llvm::LoopInfo* loopInfo)
{
    if(dom->dominates(memcpy_instr->getParent(), target_instr->getParent()) && pdom->dominates(target_instr->getParent(), memcpy_instr->getParent())
        && loopInfo->getLoopFor(memcpy_instr->getParent())==loopInfo->getLoopFor(target_instr->getParent()))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool isSameEntry(std::vector<mem_cont>vec_1, mem_cont obj)
{
    for(auto iterator = vec_1.begin(); iterator!=vec_1.end(); iterator++)
    {
        if(*iterator == obj)
        {
            return true;
        }
    }
    return false;
}


void post_process_copy(std::map<mem_cont, std::vector<mem_cont>>&opp_copy)
{
    for(auto iterator = map_to_instr.begin(); iterator!=map_to_instr.end(); iterator++)
    {
        for(auto iterator_j = iterator->second.begin(); iterator_j!=iterator->second.end(); iterator_j++)
        {
            if(isSameEntry(opp_copy[*iterator_j], iterator->first))
            {
                continue;
            }
            else
            {
                mem_cont obj = *iterator_j;
                if(opp_copy.find(obj) == opp_copy.end())
                {
                    opp_copy.insert(std::pair<mem_cont, std::vector<mem_cont>>(obj, {}));
                }
                opp_copy[obj].push_back(iterator->first);
            }
        }   
    }
}

//Get residue context from the contexts
std::vector<llvm::Instruction*> getResidue(std::vector<llvm::Instruction*>context, llvm::Function *common_caller)
{
    std::vector<llvm::Instruction*>residue;
    bool push = false;
    for(auto instr = context.begin(); instr!=context.end(); instr++)
    {
        if(push)
        {
            residue.push_back(*instr);
        }
        if((*instr)->getFunction() == common_caller)
        {
            residue.push_back(*instr);
            push = true;
        }
    }
    return residue;
}

//Find the destinatipon function of the source instruction, needed to be interprocedural analysis
llvm::Function * findDestFunction(llvm::Instruction *src, llvm::Instruction *dest, std::vector<llvm::Instruction*>src_context, 
    std::vector<llvm::Instruction*>dest_context, bool &isSourceLevel, llvm::Instruction *&return_site)
{
    isSourceLevel = true; //Pointing that the destinaion is in the source callee branch
    llvm::Function *common_caller = getCommonCaller(src_context, dest_context); //Get the common caller function
    if(common_caller == nullptr)
    {
        common_caller = src->getFunction()->getParent()->getFunction("main"); //get main function as common caller
    }
    llvm::Function *curr_func = src->getFunction(); //get the source function
    std::vector<llvm::ReturnInst*> return_set = func_to_return.at(curr_func);
    llvm::Function *dest_func = nullptr;
    bool movable = true;
    llvm::Instruction *last_instr = nullptr;
    if(src->getFunction() != common_caller)
    {
        for(auto ret = return_set.begin(); ret!=return_set.end(); ret++) //finding if source instruction can be moved out of the function or not
        {
            if(llvm::isPotentiallyReachable(src, *ret))
            {
                llvm::BasicBlock *ret_block = (*ret)->getParent();
                llvm::BasicBlock *src_block = src->getParent();
                auto dom = func_to_dom.at(src_block->getParent());
                auto pdom = func_to_pdom.at(src_block->getParent());
                if(!dom->dominates(src_block, ret_block) || !pdom->dominates(ret_block, src_block))
                {
                    movable = false;
                    break;
                }
            }
        }
        if(!movable)
        {
            dest_func = curr_func; //if not movable, then return the current function
            return dest_func;
        }
        curr_func = src_context.back()->getFunction();
        last_instr = src_context.back();
        while(curr_func != common_caller && !src_context.empty())
        {
            llvm::Instruction *src_p = src_context.back(); //Get the Last Instruction of the source context
            curr_func = src_p->getFunction(); //Get the function of the last instruction of the source context
            return_set = func_to_return.at(curr_func);
            for(auto ret = return_set.begin(); ret!=return_set.end(); ret++)
            {
                if(llvm::isPotentiallyReachable(src_p, *ret))
                {
                    llvm::BasicBlock *ret_block = (*ret)->getParent();
                    llvm::BasicBlock *src_block = src_p->getParent();
                    auto dom = func_to_dom.at(src_block->getParent());
                    auto pdom = func_to_pdom.at(src_block->getParent());
                    if(!dom->dominates(src_block, ret_block) || !pdom->dominates(ret_block, src_block))
                    {
                        return_site = src_p;
                        movable = false;
                        break;
                    }
                }
            }
            if(!movable)
            {
                dest_func = curr_func;
                return dest_func;
            }
            last_instr = src_context.back();
            src_context.pop_back();
        }
    }
    else
    {
        last_instr = src;
    }
    isSourceLevel = false; //indicating that the destination can be moved to the destination branch
    //reached at this point means can be transferred to the common caller
    std::vector<llvm::Instruction*> residue_context = getResidue(dest_context, common_caller); //Get residue context from the destination context
    

    if(residue_context.size()==0)
    {
        return_site = dest; //if residue context is empty, then return the destination
        return common_caller; //residue context is empty, then return the common caller
    }
    llvm::Instruction *dest_call_site = residue_context.front(); //Get the first instruction of the residue context
    auto dom_l = func_to_dom.at(dest_call_site->getFunction());
    auto pdom_l = func_to_pdom.at(dest_call_site->getFunction());
    if(!dom_l->dominates(last_instr, dest_call_site) || !pdom_l->dominates(dest_call_site, last_instr))
    {
        return_site = dest_call_site;
        return common_caller;
    }
    for(auto iter = residue_context.begin(); iter!=residue_context.end(); iter++)
    {
        if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(*iter))
        {
            llvm::Function *callee = CI->getCalledFunction();
            llvm::BasicBlock *entry_block = &callee->getEntryBlock();
            if(iter!=residue_context.end()-1)
            {
                llvm::Instruction *next_callsite = *(iter+1);
                llvm::BasicBlock *next_callsite_block = next_callsite->getParent();
                auto dom_p = func_to_dom.at(entry_block->getParent());
                auto pdom_p = func_to_pdom.at(entry_block->getParent());
                if(!dom_p->dominates(entry_block, next_callsite_block) || !pdom_p->dominates(next_callsite_block, entry_block))
                {
                    return_site = next_callsite;
                    movable = false;
                }
            }
            else
            {   
                llvm::BasicBlock *dest_block = dest->getParent();
                auto dom_p = func_to_dom.at(entry_block->getParent());
                auto pdom_p = func_to_pdom.at(entry_block->getParent());
                if(!dom_p->dominates(entry_block, dest_block) || !pdom_p->dominates(dest_block, entry_block))
                {
                    return_site = dest;
                    movable = false;
                }
            }
            if(!movable)
            {
                return callee;
            }
        }  
    }
    if(movable)
    {
        return_site = dest;
        return dest->getParent()->getParent();
    }
}

bool isInSuccSBB(llvm::Instruction *i, llvm::Instruction *j)
{
    llvm::BasicBlock *common_block = i->getParent();
    for(auto inst = common_block->begin(); inst!=common_block->end(); inst++)
    {
        if(llvm::Instruction *instr = llvm::dyn_cast<llvm::Instruction>(inst))
        {
            if(instr == i)
            {
                return true;
            }
            else if(instr == j)
            {
                return false;
            }
        }
    }
}

void print_result_output(Module &M, std::chrono::milliseconds duration, int total_lines, std::chrono::milliseconds duration_tot)
{
    std::string filename = "log.txt";
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
    
    // Add header with module name and date/time
    time_t now = time(0);
    char* dt = ctime(&now);
    outfile << "====================================================\n";
    outfile << "  Analysis Results for Module: " << M.getName().str() << "\n";
    outfile << "  Generated on: " << dt;
    outfile << "====================================================\n\n";
    
    // Summary statistics
    outfile << "SUMMARY:\n";
    outfile << "  Total CUDA Memcpy Operations: " << correct_map.size() << "\n";
    outfile << "  Total Code Lines Analyzed: " << total_lines << "\n";
    outfile << "  Analysis Duration: " << duration.count() << " milliseconds\n";
    
    // Detailed mapping information
    outfile << "MEMCPY TO TARGET MAPPINGS:\n";
    outfile << "----------------------------------------------------\n";
    outfile << std::left << std::setw(20) << "MEMCPY LINE" << " | " 
            << std::left << std::setw(20) << "TARGET LINE" << "\n";
    outfile << "----------------------------------------------------\n";
    
    for(auto iter = correct_map.begin(); iter != correct_map.end(); iter++)
    {
        outfile << std::left << std::setw(20) << iter->first.instr->getDebugLoc().getLine() << " | " 
                << std::left << std::setw(20) << iter->second.instr->getDebugLoc().getLine() << "\n";
    }
    
    outfile << "----------------------------------------------------\n\n";
    outfile << "End of Analysis Report\n";
    outfile.close();
}

//------------------------------- FUNCTIONS FOR TRANSFORMATION STEP --------------------------------------------


std::map<llvm::Function*, ValueToValueMapTy> FuncToVMap;//Function to VMap (VMap maps the old variables to the new argument in the cloned function)
std::map<func_cont, std::map<llvm::Value*, llvm::Value*>> FuncToMovedArgMap; //Function to Moved Argument Map (Maps the old argument to the new argument in the cloned function)
std::map<llvm::Instruction*, llvm::Instruction*> old_to_new_callsite; //store the old call site to new call site mapping
std::map<llvm::Instruction*, llvm::Instruction*> map_to_cloned_instr;
std::map<llvm::Value*, llvm::Value*> arg_equality;
//Map from original Function Instruction to the Cloned Funtion Instruction
void MapInstruction_ToCloned(llvm::Function *orig_func, llvm::Function *cloned_func, std::map<mem_cont, mem_cont> &correct_map);
//Clone the Function Call Sites
void clone_target_call_site(llvm::Instruction *call_site, llvm::Function *orig_func, std::map<llvm::Function*, std::vector<llvm::Value*>>map_to_value,
    std::map<llvm::Function*, llvm::Function*> orig_to_cloned_func, std::map<llvm::Instruction*, llvm::Instruction*> &old_to_new_callsite,
    llvm::Function *common_Caller_func, llvm::Function *src_func);

//Clone the Source Function Call Site
void clone_source_call_site(llvm::Instruction *call_site, llvm::Function *orig_func, std::map<llvm::Function*, std::vector<llvm::Value*>>map_to_value,
    std::map<llvm::Function*, llvm::Function*> orig_to_cloned_func, std::map<llvm::Instruction*, llvm::Instruction*> &old_to_new_callsite,
    llvm::Function *common_Caller_func);
//Print the VMAP fro debugging purpose for the function
// void debug_print_vmap(std::map<llvm::Function*, ValueToValueMapTy>FuncToVMap);

bool isPointerToPointer(llvm::Value *V);

//Get the Function if the current instruction is a call instruction
llvm::Function* getFunctionFromInstr(llvm::Instruction *instr);


bool isPointerToPointer(llvm::Value *V)
{
    llvm::Type *type = V->getType();
    if(llvm::PointerType *PT = llvm::dyn_cast<llvm::PointerType>(type))
    {
        llvm::Type *element_type = PT->getElementType();
        return element_type->isPointerTy();
    }
    return false;
}

llvm::Function* getFunctionFromInstr(llvm::Instruction *instr)
{
    if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(instr))
    {
        return CI->getCalledFunction();
    }
    return nullptr; //Not a call instruction
}


void memcpy_transform(llvm::Instruction *memcpy_instr, std::map<llvm::Function*, std::vector<llvm::Type*>> &map_to_type, 
            std::map<llvm::Function*, std::vector<llvm::Value*>> &map_to_value)
{
    llvm::Function *memcpy_func = memcpy_instr->getFunction();
    llvm::Module *M = memcpy_func->getParent();
    if(map_to_type.find(memcpy_func) == map_to_type.end())
    {
        std::vector<llvm::Type*> NewParamType; //New Param Type while Cloning the Function (Source Function)
        for(auto &args : memcpy_func->args())
        {
            NewParamType.push_back(args.getType());
        }
        map_to_type[memcpy_func] = NewParamType; //Here we are already storing the types of the old function into a new one (cloned)
    }
    if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(memcpy_instr))
    {
        //right now targeting cudaMemcpy Only
        //Treating only cudaMemcpyDevicetoHost
        if(CI->getCalledFunction()->getName().str() == "cudaMemcpy")
        {
            llvm::Value *f_val = get_true_value(memcpy_instr->getOperand(0));
            llvm::Value *s_val = get_true_value(memcpy_instr->getOperand(1));
            // llvm::Value *t_val = memcpy_instr->getOperand(2);
            if(!isa<llvm::Argument>(f_val))
            {
                llvm::Type *type_f_val = f_val->getType();
                map_to_type[memcpy_func].push_back(type_f_val);
                map_to_value[memcpy_func].push_back(f_val); //If not an argument then value should be alloca and need to be treated specially
            }
            if(!isa<llvm::Argument>(s_val))
            {
                llvm::Type *type_s_val = s_val->getType();
                map_to_type[memcpy_func].push_back(type_s_val);
                map_to_value[memcpy_func].push_back(s_val);
            }
        }
    }
}
void target_transform(mem_cont memcpy_instr_cont, mem_cont target_instr_cont, std::map<llvm::Function*, std::vector<llvm::Type*>> &map_to_type, 
            std::map<llvm::Function*, std::vector<llvm::Value*>> &map_to_value, llvm::Instruction *target_instr,
            std::set<llvm::Function*> &visited_target_func)
{
    llvm::Instruction *memcpy_instr = memcpy_instr_cont.instr;

    std::vector<llvm::Instruction*>src_context = memcpy_instr_cont.context_arr;
    std::vector<llvm::Instruction*>target_context = target_instr_cont.context_arr;
    
    llvm::Function *memcpy_func = memcpy_instr->getFunction();
    llvm::Function *target_func = target_instr->getFunction();

    if(visited_target_func.find(target_func) == visited_target_func.end())
    {    
        std::vector<llvm::Type*> NewParamType; //New Param Type while Cloning the Function (Source Function)
        for(auto &args : target_func->args())
        {
            NewParamType.push_back(args.getType());
        }
        map_to_type[target_func] = NewParamType; //Here we are already storing the types of the old function into a new one (cloned)
        visited_target_func.insert(target_func);
    }
    if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(memcpy_instr))
    {
        //right now targeting cudaMemcpy Only
        if(CI->getCalledFunction()->getName().str() == "cudaMemcpy")
        {
            llvm::Value *f_val = get_true_value(memcpy_instr->getOperand(0));
            llvm::Value *s_val = get_true_value(memcpy_instr->getOperand(1));
            if(!isa<llvm::Argument>(f_val))
            {
                map_to_type[target_func].push_back(f_val->getType());
                map_to_value[target_func].push_back(f_val); //If not an argument then value should be alloca and need to be treated specially
            }
            else
            {
                llvm::Value *f_val_true = get_true_value(memcpy_instr->getOperand(0), src_context);
                //find if the target function has the f_val as argument
                for(auto arg_iter = target_func->arg_begin(); arg_iter!=target_func->arg_end(); arg_iter++)
                {
                    llvm::Value *arg = llvm::dyn_cast<llvm::Value>(arg_iter);
                    //check if the argument is not a pointer then skip
                    if(arg->getType()->isPointerTy()==false)
                    {
                        continue;
                    }
                    llvm::Value *arg_true = get_true_value(arg, target_context);
                    if(arg_true == f_val_true)
                    {
                        arg_equality[f_val] = arg; //store the equality of the argument
                    }
                }
                map_to_type[target_func].push_back(f_val->getType());
                map_to_value[target_func].push_back(f_val); //If not an argument then value should be alloca and need to be treated specially
            }
            if(!isa<llvm::Argument>(s_val))
            {
                map_to_type[target_func].push_back(s_val->getType());
                map_to_value[target_func].push_back(s_val);
            }
            else
            {
                llvm::Value *s_val_true = get_true_value(memcpy_instr->getOperand(1), src_context);
                map_to_type[target_func].push_back(s_val->getType()); //s_val_true may or may not be alloca type, hence we need to load types that can be passed as param
                map_to_value[target_func].push_back(s_val_true);
            }
        }
    }
}

//can we use MapInstruction_ToCloned function before the instruction of alloca get moved
void MapInstruction_ToCloned(llvm::Function *orig_func, llvm::Function *cloned_func, std::map<mem_cont, mem_cont> &correct_map)
{
    auto bb_cloned_iter = cloned_func->begin();
    auto inst_cloned_iter = bb_cloned_iter->begin();
    for(auto bb_iter = orig_func->begin(); bb_iter!=orig_func->end(), bb_cloned_iter!=cloned_func->end(); bb_iter++, bb_cloned_iter++)
    {
        for(auto inst_iter = bb_iter->begin(), inst_cloned_iter=bb_cloned_iter->begin(); inst_iter!=bb_iter->end(), inst_cloned_iter!=bb_cloned_iter->end(); inst_iter++, inst_cloned_iter++)
        {
            // for(auto c_iter = correct_map.begin(); c_iter!=correct_map.end(); c_iter++)
            // {
                if(llvm::Instruction *I_iter = llvm::dyn_cast<llvm::Instruction>(inst_iter))
                {
                    // if(I_iter == c_iter->first.instr || I_iter == c_iter->second.instr)
                    // {
                        if(llvm::Instruction *I_iter_cloned = llvm::dyn_cast<llvm::Instruction>(inst_cloned_iter))
                        {
                            map_to_cloned_instr[I_iter] = I_iter_cloned;
                        }
                    // }
                }
            // }
        }
    }
}

//clone both source and target functions
void clone_func(std::map<mem_cont, mem_cont> correct_map, std::map<llvm::Function*, std::vector<llvm::Type*>>map_to_type,
    std::map<llvm::Function*, std::vector<llvm::Value*>>map_to_value, Module &M)
{
    std::set<llvm::Instruction*> processed_call_site; //store the processed common caller
    std::set<llvm::Function*> processed_mem_func; //store the processed mem functions
    std::set<llvm::Function*> processed_target_func; //store the processed target functions

    for(auto iter=correct_map.begin(); iter!=correct_map.end(); iter++)
    {
        llvm::Instruction *memcpy_instr = iter->first.instr;
        llvm::Instruction *dest_instr = iter->second.instr;
        if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(memcpy_instr))
        {
            if(CI->getCalledFunction()->getName().str() != "cudaMemcpy")
            {
                continue;
            }
        }

        std::vector<llvm::Instruction*>src_context = iter->first.context_arr;
        //make the contextual path
        std::vector<llvm::Instruction*>src_context_path = src_context;
        src_context_path.push_back(memcpy_instr);
        //make the destination contextual path
        std::vector<llvm::Instruction*>dest_context_path = iter->second.context_arr;
        dest_context_path.push_back(dest_instr);

        //get the source function
        llvm::Function *mem_func = memcpy_instr->getFunction();
        llvm::Function *target_func = dest_instr->getFunction();
        //call site assuming that it should be in the NCC function for now
        int com_i = getCommonCallerIndex(src_context_path, dest_context_path);
        llvm::Function *common_caller = getCommonCaller(src_context_path, dest_context_path);
        // llvm::Instruction *callsite_mem = src_context_path[com_i]; //identify source call site: Unused in this function
        // std::map<llvm::Function*, ValueToValueMapTy> FuncToVMap;//Function to VMap
        
        if(mem_func != common_caller && processed_mem_func.find(mem_func) == processed_mem_func.end())
        {
            processed_mem_func.insert(mem_func);
            //process the mem_func
            std::vector<llvm::Type*> type_vec = map_to_type.at(mem_func);

            FunctionType *NewFuncType = FunctionType::get(mem_func->getReturnType(), type_vec, false); // new function type same as old function type
            Function *newfunc = Function::Create(NewFuncType, llvm::GlobalValue::ExternalLinkage, mem_func->getName().str()+"_cloned", &M);
            
            Function::arg_iterator args = newfunc->arg_begin();
            for(auto &args_it : mem_func->args())
            {
                args->setName(args_it.getName()); //giving the names to existing function arguments 
                args++;
            }
            for(auto iter_val=map_to_value.at(mem_func).begin(); iter_val!=map_to_value.at(mem_func).end(); iter_val++)
            {
                llvm::Value *s_val = *iter_val;
                args->setName(s_val->getName().str()+"_cloned"); //giving the names to newly generated function arguments
                args++;
            }
            SmallVector<ReturnInst*, 8> returns; //always declaring returns set as SmallVector of 8
            ValueToValueMapTy VMap; //creating a value map

            args = newfunc->arg_begin(); //point at the first argument of new function
            for(auto &args_it : mem_func->args())
            {
                VMap[&args_it] = &*args; //Mapping the copied arguments using VMAP
                FuncToVMap[mem_func][&args_it] = &*args;
                args++;
            }
            CloneFunctionInto(newfunc,mem_func,VMap, CloneFunctionChangeType::LocalChangesOnly, returns);
            func_to_cloned_func[mem_func] = newfunc; //store the cloned function corresponding to the original function 
            MapInstruction_ToCloned(mem_func, newfunc, correct_map); //map the instruction to the cloned instruction
        }
        if(target_func != common_caller && processed_target_func.find(target_func) == processed_target_func.end())
        {
            processed_target_func.insert(target_func);
            //process the target_func
            std::vector<llvm::Type*> type_vec = map_to_type.at(target_func);
            FunctionType *NewFuncType = FunctionType::get(target_func->getReturnType(), type_vec, false);
            Function *newfunc = Function::Create(NewFuncType, llvm::GlobalValue::ExternalLinkage, target_func->getName().str()+"_cloned", &M);
            Function::arg_iterator args = newfunc->arg_begin();
            for(auto &args_it : target_func->args())
            {
                args->setName(args_it.getName()); //giving the names to existing function arguments 
                args++;
            }
            for(auto iter_val=map_to_value.at(target_func).begin(); iter_val!=map_to_value.at(target_func).end(); iter_val++)
            {
                llvm::Value *s_val = *iter_val;
                args->setName(s_val->getName().str()+"_cloned"); //giving the names to newly generated function arguments
                args++;
            }
            SmallVector<ReturnInst*, 8> returns; //always declaring returns set as SmallVector of 8
            ValueToValueMapTy VMap; //creating a value map
            args = newfunc->arg_begin(); //point at the first argument of new function
            for(auto &args_it : target_func->args())
            {
                VMap[&args_it] = &*args; //Mapping the copied arguments using VMAP
                FuncToVMap[target_func][&args_it] = &*args;
                args++;
            }
            for(auto iter = map_to_value[target_func].begin(); iter!=map_to_value[target_func].end(); iter++)
            {
                VMap[*iter] = &*args; //Mapping the copied arguments using VMAP
                FuncToVMap[target_func][*iter] = &*args;
                args++;
            }
            // debug_print_vmap(FuncToVMap);
            CloneFunctionInto(newfunc,target_func,VMap, CloneFunctionChangeType::LocalChangesOnly, returns);
            func_to_cloned_func[target_func] = newfunc; //store the cloned function corresponding to the original function
            MapInstruction_ToCloned(target_func, newfunc, correct_map); //map the instruction to the cloned instruction
        }
    }
}

void clone_call_sites(std::map<mem_cont, mem_cont> inter_proc_correct_map, std::map<llvm::Function*, std::vector<llvm::Value*>> map_to_value, llvm::Module &M)
{
    std::set<llvm::Instruction*> processed_callsites; //store the processed functions

    for(auto iter = inter_proc_correct_map.begin(); iter!=inter_proc_correct_map.end(); iter++)
    {
        llvm::Instruction *memcpy_instr = iter->first.instr;
        llvm::Instruction *target_instr = iter->second.instr;

        if(llvm::CallInst *CI = llvm::dyn_cast<llvm::CallInst>(memcpy_instr))
        {
            if(CI->getCalledFunction()->getName().str() != "cudaMemcpy")
            {
                continue;
            }
        }

        std::vector<llvm::Instruction*> src_contextual_path = iter->first.context_arr;
        src_contextual_path.push_back(memcpy_instr);
        std::vector<llvm::Instruction*> target_contextual_path = iter->second.context_arr;
        target_contextual_path.push_back(target_instr);

        llvm::Function *src_func = memcpy_instr->getFunction();
        llvm::Function *target_func = target_instr->getFunction();

        int com_i = getCommonCallerIndex(src_contextual_path, target_contextual_path); //get the Common Caller Index: NCC
        llvm::Function *common_caller = getCommonCaller(src_contextual_path, target_contextual_path); //get the common caller function

        llvm::Instruction *src_call_site = src_contextual_path[com_i]; //get the source call site
        llvm::Instruction *dest_call_site = target_contextual_path[com_i]; //get the destination call site

        if(src_func != common_caller && processed_callsites.find(src_call_site) == processed_callsites.end())
        {
            processed_callsites.insert(src_call_site);
            // clone source call site
            clone_source_call_site(src_call_site, src_func, map_to_value, func_to_cloned_func, old_to_new_callsite, common_caller);
        }

        if(target_func != common_caller && processed_callsites.find(dest_call_site) == processed_callsites.end())
        {
            processed_callsites.insert(dest_call_site);
            //clone target call site
            clone_target_call_site(dest_call_site, target_func, map_to_value, func_to_cloned_func, old_to_new_callsite, common_caller, src_func);
        }
    }
}

//clone the source call site
void clone_source_call_site(llvm::Instruction *call_site, llvm::Function *orig_func, std::map<llvm::Function*, std::vector<llvm::Value*>>map_to_value,
    std::map<llvm::Function*, llvm::Function*> orig_to_cloned_func, std::map<llvm::Instruction*, llvm::Instruction*> &old_to_new_callsite,
    llvm::Function *common_Caller_func)
{
    // llvm::Module *M = orig_func->getParent();//get the module of the original function
    int r_index = 1;
    // llvm::Module *M = orig_func->getParent();

    SmallVector<llvm::Value*> arguments_to_callsite;
    if(llvm::CallInst *CM = llvm::dyn_cast<llvm::CallInst>(call_site))
    {
        for(auto i=0; i<CM->getNumOperands()-1; i++){
            arguments_to_callsite.push_back(CM->getArgOperand(i)); //insert in the beginning of the vector
        }
    }
    
    std::vector<llvm::Value*>newArgs; //Vector of new arguments
    llvm::Function *newfunc = orig_to_cloned_func[orig_func]; //get the cloned function corresponding to the original function

    for(auto iter_j = map_to_value[orig_func].rbegin(); iter_j!=map_to_value[orig_func].rend(); iter_j++)
    {
        if(llvm::Instruction *iter_instr = llvm::dyn_cast<llvm::Instruction>(*iter_j))
        {
            llvm::Instruction *iter_cloned = map_to_cloned_instr[iter_instr]; //get the cloned instruction corresponding to the original instruction
            
            llvm::Instruction *deleted_cumalloc = nullptr;
            if(moved_argument.find(iter_cloned) == moved_argument.end())
            {
                llvm::Value *new_val = &*(newfunc->arg_end() - r_index);
                //create new debug info

                llvm::Instruction *entry_instr = &common_Caller_func->getEntryBlock().front();
                
                iter_cloned->replaceAllUsesWith(new_val);
                iter_cloned->moveAfter(entry_instr); //moving the alloca instructions at the very beginning of the main function


                newArgs.push_back(iter_cloned); //insert in the beginning og the vector
                moved_argument.insert(iter_cloned);

                func_cont f_obj;
                f_obj.func = orig_func;
                f_obj.context_arr = {call_site};


                FuncToMovedArgMap[f_obj][&*(newfunc->arg_end() - r_index)] = iter_cloned; //store the moved argument
                FuncToMovedArgMap[f_obj][iter_instr] = iter_cloned; //store the moved argument
                r_index++;
            } 
            else
            {
                llvm::Instruction *iter_cloned_copy = iter_cloned->clone(); //clone the instruction
                iter_cloned_copy->setName(iter_cloned->getName().str()+"_gsohc.cloned");
                iter_cloned_copy->setName(iter_cloned->getName().str()+"_gsohc.cloned");
                // iter_cloned_copy->replaceAllUsesWith(&*(newfunc->arg_end() - r_index));

                llvm::Value *old_val = &*(newfunc->arg_end() - r_index);
                // old_val->replaceAllUsesWith(iter_cloned_copy);

                llvm::Instruction *entry_instr = &common_Caller_func->getEntryBlock().front();
                iter_cloned_copy->insertAfter(entry_instr);
                newArgs.push_back(iter_cloned_copy); //insert in the beginning og the vector
                moved_argument.insert(iter_cloned_copy);

                func_cont f_obj;
                f_obj.func = orig_func;
                f_obj.context_arr = {call_site};

                FuncToMovedArgMap[f_obj][&*(newfunc->arg_end() - r_index)] = iter_cloned_copy; //store the moved argument
                FuncToMovedArgMap[f_obj][iter_instr] = iter_cloned_copy; //store the moved argument
                r_index++;
            }
        }
    }
    for(auto iter = newArgs.rbegin(); iter!=newArgs.rend(); iter++)
    {
        arguments_to_callsite.push_back(*iter); //insert in the beginning of the vector
    }
    llvm::CallInst *new_call; //new call instruction
    if(llvm::CallInst *CM = llvm::dyn_cast<llvm::CallInst>(call_site))
    {
        new_call = CallInst::Create(newfunc, arguments_to_callsite, "", CM); //create a new call instruction
        new_call->setMetadata(LLVMContext::MD_dbg, call_site->getDebugLoc()); //set debug information to the newly generated call instruction 
    }
    old_to_new_callsite[call_site] = new_call; //store the old call site to new call site mapping
}

//clone the given function call site
void clone_target_call_site(llvm::Instruction *call_site, llvm::Function *orig_func, std::map<llvm::Function*, std::vector<llvm::Value*>>map_to_value,
    std::map<llvm::Function*, llvm::Function*> orig_to_cloned_func, std::map<llvm::Instruction*, llvm::Instruction*> &old_to_new_callsite,
    llvm::Function *common_Caller_func, llvm::Function *src_func)
{

    int r_index = 1;
    // llvm::Module *M = orig_func->getParent();//debug

    SmallVector<llvm::Value*> arguments_to_callsite;
    if(llvm::CallInst *CM = llvm::dyn_cast<llvm::CallInst>(call_site))
    {
        for(auto i=0; i<CM->getNumOperands()-1; i++){
            arguments_to_callsite.push_back(CM->getArgOperand(i)); //insert in the beginning of the vector
        }
    }
    std::vector<llvm::Value*>newArgs; //Vector of new arguments
    llvm::Function *newfunc = orig_to_cloned_func[orig_func]; //get the cloned function corresponding to the original function
    for(auto iter_j = map_to_value[orig_func].rbegin(); iter_j!=map_to_value[orig_func].rend(); iter_j++)
    {
        //if is an argument
        if(llvm::Argument *arg = llvm::dyn_cast<llvm::Argument>(*iter_j))
        {
            if(arg_equality.find(arg) != arg_equality.end())
            {
                llvm::Value *arg_val = arg_equality[arg];
                //get the argument number from the original function
                if(llvm::Argument *arg_val_arg = llvm::dyn_cast<llvm::Argument>(arg_val))
                {
                    if(llvm::CallInst *CM = llvm::dyn_cast<llvm::CallInst>(call_site))
                    {
                        llvm::Value *arg_val_true = CM->getArgOperand(arg_val_arg->getArgNo());
                        if(llvm::Instruction *arg_val_true_instr = llvm::dyn_cast<llvm::Instruction>(arg_val_true))
                        {
                            //clone the instruction
                            llvm::Instruction *cloned_arg_instr = arg_val_true_instr->clone();
                            cloned_arg_instr->setName(arg_val_true_instr->getName().str()+"_gsohc.cloned");
                            cloned_arg_instr->insertBefore(call_site);
                            newArgs.push_back(cloned_arg_instr); //insert in the beginning of the vector
                        }
                    }
                }
            }
            r_index++;
        }

        else if(llvm::Instruction *iter_instr = llvm::dyn_cast<llvm::Instruction>(*iter_j))
        {
            llvm::Instruction *iter_cloned = nullptr;
            if(map_to_cloned_instr.find(iter_instr) != map_to_cloned_instr.end())
            {
                iter_cloned = map_to_cloned_instr[iter_instr]; //get the cloned instruction corresponding to the original instruction
            }
            if(iter_cloned == nullptr)
            {
                iter_cloned = iter_instr; //the instruction has not been cloned that means it is in anoter function (apart from the source function)
            }
            // iter_cloned->replaceAllUsesWith(&*(newfunc->arg_end() - r_index));
            // llvm::errs()<<*(newfunc->arg_end() - r_index)->getType()->getPointerElementType()<<"\n";//debug
            llvm::Instruction *new_arg_val;
            if(iter_cloned->getType()->getPointerElementType() == (newfunc->arg_end() - r_index)->getType()->getPointerElementType())
            {
                new_arg_val = iter_cloned;
            }
            else
            {
                llvm::BitCastInst *iter_cloned_bit = new BitCastInst(iter_cloned, (newfunc->arg_end() - r_index)->getType(), "bitcast.gsohc_", call_site);
                new_arg_val = iter_cloned_bit;
            }
            r_index++;

            newArgs.push_back(new_arg_val); //insert in the beginning og the vector
        }
    }

    for(auto iter = newArgs.rbegin(); iter!=newArgs.rend(); iter++)
    {
        arguments_to_callsite.push_back(*iter); //insert in the beginning of the vector
    }
    llvm::CallInst *new_call; //new call instruction
    if(llvm::CallInst *CM = llvm::dyn_cast<llvm::CallInst>(call_site))
    {
        new_call = CallInst::Create(newfunc, arguments_to_callsite, "", CM); //create a new call instruction
        new_call->setMetadata(LLVMContext::MD_dbg, call_site->getDebugLoc()); //set debug information to the newly generated call instruction 
    }
    old_to_new_callsite[call_site] = new_call; //store the old call site to new call site mapping
}


// ------------------------------------------------------------------------------- DESIGN PRINT ---------------------------------------------------
void printCustomMessage(const std::string& message, const std::string& type);

void printCustomMessage(const std::string& message, const std::string& type)
{
    if (type == "success") {
        llvm::errs() << GREEN << "✔ " << message << RESET <<"\n";
    }
    else if (type == "error") {
        llvm::errs() << RED << "✗ " << message << RESET <<"\n";
    }
    else if (type == "warning") {
        llvm::errs() << YELLOW << "⚠ " << message << RESET <<  "\n";
    }
    else if (type == "info") {
        llvm::errs() << BLUE << "ℹ " << message << RESET <<  "\n";
    }
    else if (type == "debug") {
        llvm::errs() << CYAN << "⚙ " << message << RESET <<  "\n";
    }
    else {
        llvm::errs() << "\033[36m✎\033[0m " << message << "\n";
    }
}