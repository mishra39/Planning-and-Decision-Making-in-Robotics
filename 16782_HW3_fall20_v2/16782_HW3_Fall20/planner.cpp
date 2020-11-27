#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <chrono>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    void set_truth(bool _truth)
    {
        this->truth = _truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }
    unordered_set<Action,ActionHasher,ActionComparator> get_actions() const
    {
        return this->actions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_init_condn()
    {
        return (this->initial_conditions);
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_condn()
    {
        return (this->goal_conditions);
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gEffect; // grounded effect
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gPC; // grounded precondition

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> getPreconditions() const
    {
        return this->gPC;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> getEffects() const
    {
        return this->gEffect;
    }


    void addEffectGrounded(GroundedCondition effect)
    {
        this->gEffect.insert(effect);
    }

    void addPCGrounded(GroundedCondition pc)
    {
        this->gPC.insert(pc);
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}


// Define the state as an object
class State
{
public:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> stateCond; // condition of the state (set of literals)
    int idx;
    int cost;
    int g_val;
    int h_val;
    bool stateValid;

    // Constructors for the class
    State(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> condition)
    {
        this->stateCond = condition;
    }

    State()
    {

    }
    bool operator==(const State stateB) const
    {
        if ((stateB.stateValid != this->stateValid) || (stateB.stateCond.size() != this->stateCond.size()))
        {
            return false;
        }

        else
        {
            return this->stateCond == stateB.stateCond;
        }
    }
};

class greaterF
{
    public:
        int operator() (const State cl, const State cr) const // cl: left cr: right
        {
            // this acts as greater in pq so we create a min-heap
            return (cl.g_val + cl.h_val) > (cr.g_val + cr.h_val);
        }
};


int heuristicCalc(const State currState, const State goal)
{
    double counter =  1; // counts the number of similar conditions
    double h_val = 0;
    counter = goal.stateCond.size();
    for (auto condn: goal.stateCond)
    {
        for (auto condnCurr : currState.stateCond)
        {
            if (condn == condnCurr)
            {
                --counter;
            }
        }
    }

    h_val =  counter;
    return h_val;
}

vector<vector<string>> findAllPermutationsUtil(vector<string> env_symbolsVect)
{
    if (env_symbolsVect.size() <= 1)
    {
        //cout << "Encountered Base Case" << endl;
        return {env_symbolsVect};
    }
    vector<vector<string>> allPerms;
    for (int i = 0 ; i < env_symbolsVect.size(); i++)
    {
        vector<string> temp(env_symbolsVect.begin(), env_symbolsVect.end());
        temp.erase(temp.begin() + i);
        auto res = findAllPermutationsUtil(temp);
        //cout << "Found Permutations without the erased found for index: " << i << endl;
        for (int j = 0 ; j < res.size(); j++)
        {
            vector<string> temp2 = res[j];
            temp2.insert(temp2.begin(), env_symbolsVect[i]);
            allPerms.push_back(temp2);
        }
    }
    return allPerms;
}

void findAllPermutationsUtil2(int level, unordered_map<int,bool> visited, vector<string> env_symbols, 
                             vector<vector<vector<string>>>& allPermutations, vector<string> symbol_args)
{
    int n = env_symbols.size();
    if (level > n)
    {
        return;
    }

    for (int i = 0; i < n; ++i)
    {
        if (visited[i] == true)
        {
            continue;
        }

        visited[i] = true;
        symbol_args.push_back(env_symbols[i]);
        allPermutations[level].push_back(symbol_args);
        findAllPermutationsUtil2(level+1, visited,env_symbols,allPermutations, symbol_args);
        visited[i] = false;
        symbol_args.pop_back();
    }
}

vector<vector<vector<string>>> findAllPermutations(vector<string> env_symbolsVect)
{
    vector<vector<vector<string>>> allPermutations;
    unordered_map<int,bool> visited;
    for (int i = 0; i < env_symbolsVect.size(); i++)
    {
        vector<vector<string>> args;
        allPermutations.push_back(args);
    }

    for (int i = 0; i < env_symbolsVect.size(); i++)
    {
        visited[i] = false;
    }
    vector<string> symbolArgs;
    findAllPermutationsUtil2(0,visited,env_symbolsVect,allPermutations, symbolArgs);
    return allPermutations;
}

list<GroundedAction> getAllGroundedActions(unordered_set<string> env_symbols, unordered_set<Action, ActionHasher, ActionComparator> env_actions)
{
    // find all the permutations of the symbols
    list<GroundedAction> allGroundedActions;
    // store the symbols in a vector
    vector<string> env_symbols_vect(env_symbols.begin(),env_symbols.end());
    // Find all the possible permutations of the symbols
    vector<vector<vector<string>>> allPermutations = findAllPermutations(env_symbols_vect);
    int count = 0;
    cout << "Printing Permutations Util2: " << endl;
    for (int i = 0; i < 4; i ++)
    {
        for (int k = 0; k < allPermutations[i].size();k++)
        {
            for (auto s:allPermutations[i][k])
            {
               // cout << s << ", ";
            }
            count++;
            //cout << endl;
        }
    }
    cout << "Total Permutations Util2: " << count << endl;
    // Step 2: Create Grounded Actions using symbols permutations and environment actions
    for (auto action:env_actions)
    {
        int size = action.get_args().size()-1;
        for (auto symbolVect:allPermutations[size])
        {
            list<string> newArgs(symbolVect.begin(),symbolVect.end());
            GroundedAction newAction(action.get_name(), newArgs);
            unordered_map<string, string> arg_map;
            auto arg_val = newArgs.begin();

            for (auto arg_name: action.get_args())
            {
                arg_map[arg_name] = *arg_val;
                arg_val++;
            }

            for (auto symbol:env_symbols)
            {
                arg_map[symbol] = symbol;
            }

            // Ground Preconditions
            for (auto pc:action.get_preconditions())
            {
                list<string> pc_args;
                for (auto pc_arg : pc.get_args())
                {
                    pc_args.push_back(arg_map[pc_arg]);
                }
                GroundedCondition gPC(pc.get_predicate(), pc_args, pc.get_truth());
                newAction.addPCGrounded(gPC);
            }

            // Ground Effects
            for (auto effect : action.get_effects())
            {
                list<string> effect_args;
                for (auto effect_arg : effect.get_args())
                {
                    effect_args.push_back(arg_map[effect_arg]);
                }
                GroundedCondition gEffect(effect.get_predicate(), effect_args, effect.get_truth());
                newAction.addEffectGrounded(gEffect);
            }
                //cout << "New Action Created: " << newAction << endl;
                allGroundedActions.push_back(newAction);
        }
    }
    
    return allGroundedActions;
}
list<GroundedAction> planner(Env* env)
{
    // Use env to get all the actions
    // Ground the actions. A condition or action is defined in terms of arbitrary arguments,
    // while a grounded condition or action is defined in terms of the actual argument values, or the symbols.
    list<GroundedAction> actions;
    list<GroundedAction> actions_list =  getAllGroundedActions(env->get_symbols(), env->get_actions());
    cout << "Size of grounded actions " << actions_list.size() << endl;
    // priority queue for actions
    priority_queue<State, vector<State>, greaterF> open_pq;
    unordered_map<int, bool> closed;
    vector<State> visited;
    unordered_map<int,State> graph;
    unordered_map<int,GroundedAction> action_map;
    unordered_map<int,State> parent;
    int stateExpanded = 0; // counter for states expanded
    State currState, adjState, parentState;
    // Use GroundedAction method to get preconditions and effects
    // this is where you insert your planner
    // add initial and goal conditions from the environment grounded condition
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> init_cond = env->get_init_condn();
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_cond = env->get_goal_condn();
    State start(init_cond);
    State goal(goal_cond);
    start.g_val = 0;
    start.idx  = 0;
    goal.idx = -1;
    start.h_val = heuristicCalc(start,goal);
    open_pq.push(start);
    graph[start.idx] = start;
    auto startTime = std::chrono::steady_clock::now();
    while (!open_pq.empty())
    {
        currState = open_pq.top();
        open_pq.pop();

        //check if state is closed
        if (closed[currState.idx] == true)
        {
            continue;
        }
        closed[currState.idx] = true;

        // check if current state is goal
        if (currState.h_val == 0)
        {
            cout << "Goal found. Initiating Backtracking..."<< endl;
            break;
        }

        for (auto action:actions_list)
        {
            adjState.stateCond = currState.stateCond;
            // check if preconditions match
            for (auto pc:action.getPreconditions())
            {
                adjState.stateValid = true;
                if (currState.stateCond.find(pc) == currState.stateCond.end())
                {
                    adjState.stateValid = false;
                    break;
                }
            }

            // if preconditions match, insert the effects
            if (adjState.stateValid)
            {
                for (auto effect:action.getEffects())
                {
                    if (effect.get_truth())
                    {
                        adjState.stateCond.insert(effect);
                    }
                    else
                    {
                        effect.set_truth(true);
                        adjState.stateCond.erase(effect);
                    }
                }

                // check if neighbor has been visited already
                int neighborIdx = -1;
                for (int i = 0; i < visited.size(); i++)
                {
                    if (visited[i] == adjState)
                    {
                        neighborIdx = i;
                        break;
                    }
                }
                stateExpanded++;
                if (neighborIdx >=0)
                {
                    // check for g value update
                    if (!closed[neighborIdx] && graph[neighborIdx].g_val > currState.g_val + 1)
                    {
                        graph[neighborIdx].g_val = currState.g_val + 1;
                        open_pq.push(graph[neighborIdx]);
                        parent[graph[neighborIdx].idx] = currState;
                       // cout << action << endl;
                        action_map.at(neighborIdx) = action; 
                    }
                }

                else // if the state has never been visited before
                {
                    adjState.idx = graph.size();
                    adjState.g_val = currState.g_val + 1;
                    adjState.h_val = heuristicCalc(adjState,goal);
                    parent[adjState.idx] = currState;
                    graph[adjState.idx] = adjState;
                    action_map.insert({adjState.idx,action});
                  //  cout << action << endl;
                    visited.push_back(adjState);
                    open_pq.push(adjState);
                }
            }
        }

    }
    auto end = chrono::steady_clock::now();
    chrono::duration<double> elapsed_seconds = end-startTime;
    cout << "Total States Expanded: " << graph.size() << endl;
    cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
    vector<int> finalPath;
    while(1)
    {
        cout << currState.idx << endl;
        finalPath.push_back(currState.idx);
        if (parent[currState.idx].idx == 0)
        {
            break;
        }
        
        currState = parent[currState.idx];
    }

    reverse(finalPath.begin(),finalPath.end());
    for (auto i:finalPath)
    {
        actions.push_back(action_map.at(i));
    }
    
    return actions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    
    /*for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }*/

    return 0;
}