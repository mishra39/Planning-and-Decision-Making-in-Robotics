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

    GroundedAction(const GroundedAction& ga)
    {
        this->name = ga.name;
        for (string ar:arg_values)
        {
            this->arg_values.push_back(ar);
        }
        this->gEffect = ga.gEffect;
        this->gPC = ga.gPC;
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
// function to match a given grounded action preconditions with a condition
bool matchActionPCs(GroundedAction currAction, unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> condn)
{
    for (auto pc: currAction.getPreconditions())
    {
        if (condn.find(pc) == condn.end())
        {
            return false;
        }
    }
    return true;
}

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
    State* parent;
    bool stateValid;

    // Constructors for the class
    State(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> condition)
    {
        this->stateCond = condition;
        this->parent = NULL;
    }

    State()
    {
        this->parent = NULL;
    }

    // comparator for current state and a given state states
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

void dfs(int level, unordered_map<int,bool> visited,vector<string> all_symbols, vector<list<list<string>>>& symbPerms,list<string> arg_vals)
{ 
    // DFS finds all size of permutations of the symbol arguments
    int n = all_symbols.size();

    //  Base Case
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
        arg_vals.push_back(all_symbols[i]);
        symbPerms[level].push_back(arg_vals);
        dfs(level + 1, visited, all_symbols, symbPerms,arg_vals);

        visited[i] = false;
        arg_vals.pop_back();
    }
}

void getPerms(vector<list<list<string>>>& symPerms, const unordered_set<string> symbols)
{
    vector<string> all_symbols;
    unordered_map<int,bool> visited;
    list<string> arg_vals;
   
    for (auto i:symbols)
    {
        all_symbols.push_back(i);
    }
    int n = all_symbols.size();
    for (int i = 0; i < n; ++i)
    { // Populate vector with 2D string lists
        list<list<string>> perms;
        symPerms.push_back(perms);
    }
    for (int i = 0; i <n; i++)
    {
        visited[i] = false;
    }
    dfs(0,visited,all_symbols,symPerms,arg_vals);
    cout << "DFS completed" << endl;
}
list<GroundedAction> getGroundedActions(const unordered_set<string> env_symbols, const unordered_set<Action, ActionHasher,ActionComparator> env_actions)
{
    list<GroundedAction> groundedActions; // final list of grounded actions
    vector<list<list<string>>> symPerms; // permutations of the symbols
    getPerms(symPerms, env_symbols);

    for (auto action: env_actions)
    {
        int size = action.get_args().size()-1;
        for (auto currArgs: symPerms[size])
        {
            auto k = currArgs.begin();
            // Create a grounded action for this set of symbols and actions
            GroundedAction currGA(action.get_name(),currArgs);
            unordered_map<string,string> arg_map;

            for (auto i:env_symbols)
            {
                arg_map[i] = i;
            }
            
            for (auto s:action.get_args())
            {
                arg_map[s] = *k;
                ++k;
            }

            // assign values (ground) to the effects
            for (auto effect:action.get_effects())
            {
                list<string> arg_list;
                for (auto arg_effect: effect.get_args())
                {
                    arg_list.push_back(arg_map[arg_effect]);
                }
                GroundedCondition gEffect(effect.get_predicate(),arg_list, effect.get_truth());
                currGA.addEffectGrounded(gEffect);
            }
            // assign values (ground) to the effects
            for (auto pc:action.get_preconditions())
            {
                list<string> arg_list;
                for (auto pArg: pc.get_args())
                {
                    arg_list.push_back(arg_map[pArg]);
                }
                GroundedCondition gPC(pc.get_predicate(),arg_list, pc.get_truth());
                currGA.addPCGrounded(gPC);
            }
            groundedActions.emplace_back(currGA);
        }
    }
    return groundedActions;
}

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

void findNeighbor(const State currState, GroundedAction action, State& adjNode)
{
    State neighbor(currState.stateCond);
    neighbor.stateValid = matchActionPCs(action, currState.stateCond);
    // check if preconditions match
    if (neighbor.stateValid)
    {
        for (auto effect:action.getEffects())
        {
            if (effect.get_truth())
            {
                neighbor.stateCond.insert(effect);
                //cout << "Inserting Effect " << effect << endl;
            }
            else
            {
                effect.set_truth(true);
               // cout << "Erasing effect " << effect << endl;
                neighbor.stateCond.erase(effect);
            }
        }
    }
    adjNode = neighbor;
}

list<GroundedAction> planner(Env* env)
{
    // Use env to get all the actions
    // Ground the actions. A condition or action is defined in terms of arbitrary arguments,
    // while a grounded condition or action is defined in terms of the actual argument values, or the symbols.
    list<GroundedAction> actions_list = getGroundedActions(env->get_symbols(), env->get_actions());
    cout << "Size of grounded actions " << actions_list.size() << endl;
    // priority queue for actions
    priority_queue<State, vector<State>, greaterF> open_pq;
    unordered_map<int, bool> closed;
    vector<State> visited;
    unordered_map<int,State> graph;
    unordered_map<int,GroundedAction> action_map;
    State currState, adjState, parentState;
    //priority_queue<Node, vector<Node>, greater> > open_list;
    // Use GroundedAction method to get preconditions and effects
    // this is where you insert your planner
    // add initial and goal conditions from the environment grounded condition
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> init_cond = env->get_init_condn();
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_cond = env->get_goal_condn();
    
    State start(init_cond);
    State goal(goal_cond);
    start.g_val = 0;
    start.idx  = 0;
    start.h_val = heuristicCalc(start,goal);
    open_pq.push(start);
    graph[start.idx] = start;
    int count = 0;
    // Loop through all actions in the environment to create all possible action combination with symbols
    list<GroundedAction> actions;
    while(!open_pq.empty())
    {
        cout << "Couter Expansion: " << ++count << endl;
        currState = open_pq.top();
        open_pq.pop();

        // if state is closed, then skip it
        if (closed[currState.idx] == true)
        {
            continue;
        }

        closed[currState.idx] = true;

        if (currState.h_val == 0)
        {
            cout << "Goal found" << endl;
            break;
        }

        // iterate over the actions
        for (auto action : actions_list)
        {
            //findNeighbor(currState,action,adjState);
            adjState.stateCond = currState.stateCond;
            adjState.stateValid = matchActionPCs(action, currState.stateCond);
            // check if preconditions match
            if (adjState.stateValid)
            {
                for (auto effect:action.getEffects())
                {
                    if (effect.get_truth())
                    {
                        adjState.stateCond.insert(effect);
                        //cout << "Inserting Effect " << effect << endl;
                    }
                    else
                    {
                        effect.set_truth(true);
                    // cout << "Erasing effect " << effect << endl;
                        adjState.stateCond.erase(effect);
                    }
                }
            }

            if (adjState.stateValid)
            {
                // check if visited already
                int visited_val = -1;
                for (int i = 0; i < visited.size();++i)
                {
                    if (visited[i] == adjState)
                    {
                        visited_val = i;
                        break;
                    }
                }

                if (visited_val != -1)
                {
                    if (closed[visited_val] == false && (graph[visited_val].g_val > currState.g_val + 1))
                    {
                        // Check if g values can be updated
                        graph[visited_val].g_val = currState.g_val + 1;
                        //parent;
                        open_pq.push(graph[visited_val]);
                        action_map.at(visited_val) = action;
                    }
                }

                else
                {
                    adjState.g_val = currState.cost + 1;
                    adjState.h_val = heuristicCalc(adjState, goal);
                    adjState.idx = graph.size();
                    //adjState.parent = &currState;
                    graph[adjState.idx] = adjState;
                    visited.push_back(adjState);
                    open_pq.push(adjState);
                    action_map.insert({graph.size(),action});
                }

                cout << "Previous State: ";
                for (auto cond:currState.stateCond)
                {
                    cout << cond;
                }

                cout << endl;

                cout << "Current Action Applied " << action << endl;
                cout << "New State: " ;
                for (auto cond:adjState.stateCond)
                {
                    cout << cond ;
                }

                cout << endl;
                cout << endl;
                cout << endl;
                cout << endl;
            }
        }

    }

    actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

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
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}