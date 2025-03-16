// comp := g++ data.cpp /home/jyh/.local/include/cpglib/print.o -o data -O1 -std=gnu++20 -Wall -Wextra -Wshadow -D_GLIBCXX_ASSERTIONS -fmax-errors=2 -DLOCAL
// run  := ./data 190 30 1.5 3 dp 3
// dir  := .
// kid  :=
#include <bits/stdc++.h>
#include <format>
#include <cassert>
using namespace std;
#pragma GCC optimize("unroll-loops")
#pragma GCC target("avx2,bmi,bmi2,lzcnt,popcnt")
#ifdef LOCAL
#include <cpglib/print.h>
#define debug(x...) _debug_print(0, #x, x);
#define Debug(x...) _debug_print(1, #x, x);
#define DEBUG(x...) _debug_print(2, #x, x);
std::ifstream terminal("/dev/tty");
#define PP cerr<<"\033[1;33mpause...\e[0m",terminal.ignore();
#else
#define debug(x...)
#define Debug(x...)
#define DEBUG(x...)
#define PP
#endif
template<typename...Args> void print_(Args...args){((cout<<args<<" "),...)<<endl;}
#define rep(i,a,b) for(int i=(a);i<(int)(b);++i)
#define sz(v) ((int)(v).size())
#define print(...) print_(__VA_ARGS__);
#define FIND(a, x) ((find(a.begin(),a.end(),(x))!=a.end())?1:0)
#define cmin(x,...) x=min({(x),__VA_ARGS__})
#define cmax(x,...) x=max({(x),__VA_ARGS__})
#define INTMAX (int)(9223372036854775807)
#define INF (int)(1152921504606846976)
#define NaN (int)(0x8b88e1d0595d51d1)
#define double long double
#define int long long
#define uint unsigned long long
#define endl "\n"

#define LEVEL_MAX 2ll
#define LEVEL_ALL (LEVEL_MAX*2+1)

enum class Action {
    BAIT = 0,
    MUD = 1,
    BALL = 2
};

string decision_name(Action a) {
    if (a == Action::BAIT) return "Throw Bait";
    else if (a == Action::MUD) return "Throw Mud";
    else if (a == Action::BALL) return "Throw Ball";
    else assert(false);
}

enum class State {
    ACTIVE,
    NOBALL,
    FLED,
    CAPTURED
};

struct Graph {

    Graph() = default;

    struct Vertex {
        int ball_cnt;
        int capture_level;
        int flee_level;
        State state;

        Vertex(int bc, int cl, int fl) {
            ball_cnt = bc;
            capture_level = cl;
            flee_level = fl;
            state = State::ACTIVE;
        }
        Vertex(State s) {
            ball_cnt = capture_level = flee_level = -1;
            state = s;
        }
        Vertex() = default;

        array<vector<pair<double, Vertex*>>, 3> nxt;
    };

    vector<Vertex> vertices;
    Vertex *start, *fled, *captured, *noball;
    int start_idx;

    int get_idx(int bc, int cl, int fl) {
        assert(bc > 0);
        return (bc-1)*LEVEL_ALL*LEVEL_ALL + (cl+LEVEL_MAX)*LEVEL_ALL + (fl+LEVEL_MAX);
    }

    int get_idx(Vertex &v) {
        if (v.state == State::FLED) return (int)vertices.size()-3;
        else if (v.state == State::CAPTURED) return (int)vertices.size()-2;
        else if (v.state == State::NOBALL) return (int)vertices.size()-1;
        else return get_idx(v.ball_cnt, v.capture_level, v.flee_level);
    }

    bool is_terminal(Vertex &v) {
        return v.state == State::FLED || v.state == State::CAPTURED || v.state == State::NOBALL;
    }

    Graph(int ibc, function<double(int)> capture_prob, function<double(int)> flee_prob) {
        vertices.resize(LEVEL_ALL * LEVEL_ALL * ibc + 3);
        for (int bc = 1; bc <= ibc; ++bc) {
            for (int cl = -LEVEL_MAX; cl <= LEVEL_MAX; ++cl) {
                for (int fl = -LEVEL_MAX; fl <= LEVEL_MAX; ++fl) {
                    vertices[get_idx(bc, cl, fl)] = Vertex(bc, cl, fl);
                }
            }
        }
        start_idx = get_idx(ibc, 0, 0);
        this->start = &vertices[start_idx];

        vertices[LEVEL_ALL*LEVEL_ALL*ibc].state = State::FLED;
        fled = &vertices[LEVEL_ALL*LEVEL_ALL*ibc];

        vertices[LEVEL_ALL*LEVEL_ALL*ibc+1].state = State::CAPTURED;
        captured = &vertices[LEVEL_ALL*LEVEL_ALL*ibc+1];

        vertices[LEVEL_ALL*LEVEL_ALL*ibc+2].state = State::NOBALL;
        noball = &vertices[LEVEL_ALL*LEVEL_ALL*ibc+2];

        for (int i = 0; i < LEVEL_ALL*LEVEL_ALL*ibc; ++i) build(&vertices[i], capture_prob, flee_prob);

        shrink();
    }

    bool is_zero(double x) {
        return abs(x) < 1e-9;
    }

    void add_edge(Vertex *u, Vertex *v, double p, Action a) {
        if (is_zero(p)) return;
        u->nxt[(int)a].push_back({p, v});
    }

    void shrink() {
        for (Vertex &u: vertices) {
            for (int ac = 0; ac < 3; ++ac) {
                vector<pair<int, double>> orig;
                for (auto [p, v]: u.nxt[ac]) orig.push_back({get_idx(*v), p});
                sort(orig.begin(), orig.end());
                int v_prev = -1;
                double total_p = 0;
                vector<pair<double, Vertex*>> nxt_new;
                for (auto [v, p]: orig) {
                    if (v == v_prev) total_p += p;
                    else {
                        if (v_prev != -1) nxt_new.push_back({total_p, &vertices[v_prev]});
                        v_prev = v;
                        total_p = p;
                    }
                }
                if (!is_zero(total_p)) nxt_new.push_back({total_p, &vertices[v_prev]});
                u.nxt[ac] = nxt_new;

                // TEST: total p = 1
                if (u.nxt[ac].size() > 0) {
                    total_p = 0;
                    for (auto [p, v]: u.nxt[ac]) total_p += p;
                    assert(abs(total_p - 1) < 1e-9);
                }
            }
        }
    }

    void build(Vertex *v, function<double(int)> capture_prob, function<double(int)> flee_prob) {

        function<int(int)> level_add = [](int level) -> int {
            return min(level + 1, LEVEL_MAX);
        };
        function<int(int)> level_sub = [](int level) -> int {
            return max(level - 1, -LEVEL_MAX);
        };

        if (is_terminal(*v)) assert(false);
        int bc = v->ball_cnt;
        int cl = v->capture_level;
        int fl = v->flee_level;
        // DONE: BAIT
        Vertex *new_state_bait_90 = &vertices[get_idx(bc, level_sub(cl), level_sub(fl))];
        double flee_rate_bait_90 = flee_prob(new_state_bait_90->flee_level);
        Vertex *new_state_bait_10 = &vertices[get_idx(bc, cl, level_sub(fl))];
        double flee_rate_bait_10 = flee_prob(new_state_bait_10->flee_level);
        add_edge(v, new_state_bait_90, 0.9 * (1 - flee_rate_bait_90), Action::BAIT);
        add_edge(v, fled, 0.9 * flee_rate_bait_90, Action::BAIT);
        add_edge(v, new_state_bait_10, 0.1 * (1 - flee_rate_bait_10), Action::BAIT);
        add_edge(v, fled, 0.1 * flee_rate_bait_10, Action::BAIT);

        // DONE: MUD
        Vertex *new_state_mud_90 = &vertices[get_idx(bc, level_add(cl), level_add(fl))];
        double flee_rate_mud_90 = flee_prob(new_state_mud_90->flee_level);
        Vertex *new_state_mud_10 = &vertices[get_idx(bc, level_add(cl), fl)];
        double flee_rate_mud_10 = flee_prob(new_state_mud_10->flee_level);
        add_edge(v, new_state_mud_90, 0.9 * (1 - flee_rate_mud_90), Action::MUD);
        add_edge(v, fled, 0.9 * flee_rate_mud_90, Action::MUD);
        add_edge(v, new_state_mud_10, 0.1 * (1 - flee_rate_mud_10), Action::MUD);
        add_edge(v, fled, 0.1 * flee_rate_mud_10, Action::MUD);

        // DONE: BALL
        double capture_rate = capture_prob(cl);
        add_edge(v, captured, capture_rate, Action::BALL);
        double flee_rate_capture = flee_prob(fl);
        add_edge(v, fled, (1 - capture_rate) * flee_rate_capture, Action::BALL);
        Vertex *not_fled = bc > 1 ? &vertices[get_idx(bc-1, cl, fl)] : noball;
        add_edge(v, not_fled, (1 - capture_rate) * (1 - flee_rate_capture), Action::BALL);
    }

    void naive() {
        function<double(Vertex*, double, int)> dp = [&](Vertex *u, double p_acc, int depth) -> double {
            if (p_acc < 1e-2) return 0;

            if (u->state == State::FLED) return 0;
            else if (u->state == State::CAPTURED) return 1;
            else if (u->state == State::NOBALL) return 0;

            // DONE: BALL
            double prob_ball = 0;
            for (auto [p, v]: u->nxt[(int)Action::BALL]) {
                prob_ball += p * dp(v, p_acc * p, depth + 1);
            }
            // DONE: BAIT
            double prob_bait = 0;
            for (auto [p, v]: u->nxt[(int)Action::BAIT]) {
                prob_bait += p * dp(v, p_acc * p, depth + 1);
            }
            // DONE: MUD
            double prob_mud = 0;
            for (auto [p, v]: u->nxt[(int)Action::MUD]) {
                prob_mud += p * dp(v, p_acc * p, depth + 1);
            }

            return max({prob_ball, prob_bait, prob_mud});
        };
        double result = dp(start, 1.0, 1);
        debug(result);
    }

    void search() {
        vector<bool> vis(vertices.size());
        function<double(Vertex*, double, int)> dp = [&](Vertex *u, double p_acc, int depth) -> double {
            vis[get_idx(*u)] = true;

            if (p_acc < 1e-5) return 0;

            if (u->state == State::FLED) return 0;
            else if (u->state == State::CAPTURED) return 1;
            else if (u->state == State::NOBALL) return 0;

            // DONE: BALL
            double prob_ball = 0;
            bool valid_ball = 0;
            for (auto [p, v]: u->nxt[(int)Action::BALL]) {
                if (!vis[get_idx(*v)]) {
                    valid_ball = 1;
                    break;
                }
            }
            if (valid_ball) {
                for (auto [p, v]: u->nxt[(int)Action::BALL]) {
                    prob_ball += p * dp(v, p_acc * p, depth + 1);
                }
            }
            // DONE: BAIT
            double prob_bait = 0;
            bool valid_bait = 0;
            for (auto [p, v]: u->nxt[(int)Action::BAIT]) {
                if (!vis[get_idx(*v)]) {
                    valid_bait = 1;
                    break;
                }
            }
            if (valid_bait) {
                for (auto [p, v]: u->nxt[(int)Action::BAIT]) {
                    prob_bait += p * dp(v, p_acc * p, depth + 1);
                }
            }
            // DONE: MUD
            double prob_mud = 0;
            bool valid_mud = 0;
            if (valid_mud) {
                for (auto [p, v]: u->nxt[(int)Action::MUD]) {
                    prob_mud += p * dp(v, p_acc * p, depth + 1);
                }
            }

            vis[get_idx(*u)] = false;
            return max({prob_ball, prob_bait, prob_mud});
        };
        double result = dp(start, 1.0, 1);
        debug(result);
    }

    void linear_programming(string filename = "lplp.py") {

        function<void(ofstream &, vector<pair<double, int>>)> print_equation = [&print_equation](ofstream &fout, vector<pair<double, int>> equations) {
            if (equations.size() == 1) {
                fout << equations[0].first << "*p" << equations[0].second << "\n";
                return;
            }
            fout << equations[0].first << "*p" << equations[0].second << " + ";
            equations = vector<pair<double, int>>(equations.begin()+1, equations.end());
            print_equation(fout, equations);
        };

        ofstream fout(filename);
        fout << "import pulp\n";
        fout << "\n";
        fout << "prob = pulp.LpProblem('Pokemon', pulp.LpMinimize)\n";
        fout << "\n";

        for (Vertex &u: vertices) {
            int idx_u = get_idx(u);
            fout << "p" << idx_u << " = pulp.LpVariable('p" << idx_u << "', lowBound=0, upBound=1, cat='Continuous')\n";
        }
        fout << "\n";
        fout << "prob += p" << start_idx << ", 'Minimize_pstart'\n";
        fout << "\n";

        for (Vertex &u: vertices) {
            vector<pair<double, int>> equations;
            int idx_u = get_idx(u);
            if (u.state == State::FLED || u.state == State::CAPTURED || u.state == State::NOBALL) {
                if (u.state == State::FLED || u.state == State::NOBALL) fout << "prob += p" << idx_u << " == 0\n";
                else if (u.state == State::CAPTURED) fout << "prob += p" << idx_u << " == 1\n";
                continue;
            }
            // DONE: BAIT
            equations.clear();
            for (auto [p, v]: u.nxt[(int)Action::BAIT]) {
                equations.push_back({p, get_idx(*v)});
            }
            if (equations.size() > 0) {
                fout << "prob += p" << idx_u << " >= ";
                print_equation(fout, equations);
            }

            // DONE: MUD
            equations.clear();
            for (auto [p, v]: u.nxt[(int)Action::MUD]) {
                equations.push_back({p, get_idx(*v)});
            }
            if (equations.size() > 0) {
                fout << "prob += p" << idx_u << " >= ";
                print_equation(fout, equations);
            }

            // DONE: BALL
            equations.clear();
            for (auto [p, v]: u.nxt[(int)Action::BALL]) {
                equations.push_back({p, get_idx(*v)});
            }
            if (equations.size() > 0) {
                fout << "prob += p" << idx_u << " >= ";
                print_equation(fout, equations);
            }
        }

        fout << "\n";
        fout << "status = prob.solve(pulp.PULP_CBC_CMD(msg=0))\n";
        fout << "print(pulp.LpStatus[status])\n";
        fout << "print(p" << start_idx << ".varValue)\n";
    }

    void graphviz_dump(string filename) {
        ofstream fout(filename);
        fout << "digraph {\n";

        for (Vertex &u: vertices) {
            int idx_u = get_idx(u);
            if (u.state == State::FLED || u.state == State::CAPTURED || u.state == State::NOBALL) {
                if (u.state == State::FLED) fout << format("    v{}[shape=\"record\" label=\"v{}|FLED\"];\n", idx_u, idx_u);
                else if (u.state == State::CAPTURED) fout << format("    v{}[shape=\"record\" label=\"v{}|CAPTURED\"];\n", idx_u, idx_u);
                else if (u.state == State::NOBALL) fout << format("    v{}[shape=\"record\" label=\"v{}|NOBALL\"];\n", idx_u, idx_u);
                continue;
            }
            fout << format("    v{}[shape=\"record\" label=\"v{}|ball={}|cap={}|flee={}\"];\n", idx_u, idx_u, u.ball_cnt, u.capture_level, u.flee_level);
        }

        fout << "Legend [shape=none, margin=0, label=<\n";
            fout << "<TABLE BORDER=\"0\" CELLBORDER=\"1\" CELLSPACING=\"0\" CELLPADDING=\"4\">\n";
                fout << "<TR><TD COLSPAN=\"2\"><B>Legend</B></TD></TR>\n";
                fout << "<TR>\n";
                    fout << "<TD><FONT COLOR=\"red\">&#10148;</FONT></TD>\n";
                    fout << "<TD>throw ball</TD>\n";
                fout << "</TR>\n";
                fout << "<TR>\n";
                    fout << "<TD><FONT COLOR=\"green\">&#10148;</FONT></TD>\n";
                    fout << "<TD>throw bait</TD>\n";
                fout << "</TR>\n";
                fout << "<TR>\n";
                    fout << "<TD><FONT COLOR=\"blue\">&#10148;</FONT></TD>\n";
                    fout << "<TD>throw mud</TD>\n";
                fout << "</TR>\n";
            fout << "</TABLE>\n";
        fout << ">];\n";

        for (Vertex &u: vertices) {
            vector<pair<double, int>> equations;
            int idx_u = get_idx(u);
            for (auto [p, v]: u.nxt[(int)Action::BALL]) {
                int idx_v = get_idx(*v);
                fout << format("    v{} -> v{} [label=\"{:.4f}\", color=\"red\"];\n", idx_u, idx_v, p);
            }
            for (auto [p, v]: u.nxt[(int)Action::BAIT]) {
                int idx_v = get_idx(*v);
                fout << format("    v{} -> v{} [label=\"{:.4f}\", color=\"green\"];\n", idx_u, idx_v, p);
            }
            for (auto [p, v]: u.nxt[(int)Action::MUD]) {
                int idx_v = get_idx(*v);
                fout << format("    v{} -> v{} [label=\"{:.4f}\", color=\"blue\"];\n", idx_u, idx_v, p);
            }
        }

        fout << "}\n";
    }
};

struct GraphOpt {

    GraphOpt() = default;

    struct Vertex {
        int ball_cnt;
        int capture_level;
        int flee_level;
        int round;
        State state;

        Vertex(int bc, int cl, int fl, int r) {
            ball_cnt = bc;
            capture_level = cl;
            flee_level = fl;
            round = r;
            state = State::ACTIVE;
        }
        Vertex(State s) {
            ball_cnt = capture_level = flee_level = round = -1;
            state = s;
        }
        Vertex() = default;

        array<vector<pair<double, Vertex*>>, 3> nxt;

        bool operator <(const Vertex &v) const {
            return tie(ball_cnt, capture_level, flee_level, round) < tie(v.ball_cnt, v.capture_level, v.flee_level, v.round);
        }
    };

    vector<Vertex> vertices;
    Vertex *start, *fled, *captured, *noball;
    int start_idx;
    int round_max;
    function<double(int)> P_caught, P_fled;

    int get_idx(int bc, int cl, int fl, int r) {
        assert(bc > 0);
        return (bc-1)*LEVEL_ALL*LEVEL_ALL*round_max + (cl+LEVEL_MAX)*LEVEL_ALL*round_max + (fl+LEVEL_MAX)*round_max + r;
    }

    int get_idx(Vertex &v) {
        if (v.state == State::FLED) return (int)vertices.size()-3;
        else if (v.state == State::CAPTURED) return (int)vertices.size()-2;
        else if (v.state == State::NOBALL) return (int)vertices.size()-1;
        else return get_idx(v.ball_cnt, v.capture_level, v.flee_level, v.round);
    }

    bool is_terminal(Vertex &v) {
        return v.state == State::FLED || v.state == State::CAPTURED || v.state == State::NOBALL;
    }

    GraphOpt(int ibc, int rm, function<double(int)> capture_prob, function<double(int)> flee_prob) {
        round_max = rm;
        P_caught = capture_prob;
        P_fled = flee_prob;
        vertices.resize(round_max * LEVEL_ALL * LEVEL_ALL * ibc + 3);
        for (int bc = 1; bc <= ibc; ++bc) {
            for (int cl = -LEVEL_MAX; cl <= LEVEL_MAX; ++cl) {
                for (int fl = -LEVEL_MAX; fl <= LEVEL_MAX; ++fl) {
                    for (int r = 0; r < round_max; ++r) {
                        vertices[get_idx(bc, cl, fl, r)] = Vertex(bc, cl, fl, r);
                    }
                }
            }
        }
        start_idx = get_idx(ibc, 0, 0, 0);
        this->start = &vertices[start_idx];

        vertices[round_max*LEVEL_ALL*LEVEL_ALL*ibc].state = State::FLED;
        fled = &vertices[round_max*LEVEL_ALL*LEVEL_ALL*ibc];

        vertices[round_max*LEVEL_ALL*LEVEL_ALL*ibc+1].state = State::CAPTURED;
        captured = &vertices[round_max*LEVEL_ALL*LEVEL_ALL*ibc+1];

        vertices[round_max*LEVEL_ALL*LEVEL_ALL*ibc+2].state = State::NOBALL;
        noball = &vertices[round_max*LEVEL_ALL*LEVEL_ALL*ibc+2];

        for (int i = 0; i < round_max*LEVEL_ALL*LEVEL_ALL*ibc; ++i) build(&vertices[i], capture_prob, flee_prob);

        shrink();
    }

    bool is_zero(double x) {
        return abs(x) < 1e-9;
    }

    void add_edge(Vertex *u, Vertex *v, double p, Action a) {
        if (is_zero(p)) return;
        u->nxt[(int)a].push_back({p, v});
    }

    void shrink() {
        for (Vertex &u: vertices) {
            for (int ac = 0; ac < 3; ++ac) {
                vector<pair<int, double>> orig;
                for (auto [p, v]: u.nxt[ac]) orig.push_back({get_idx(*v), p});
                sort(orig.begin(), orig.end());
                int v_prev = -1;
                double total_p = 0;
                vector<pair<double, Vertex*>> nxt_new;
                for (auto [v, p]: orig) {
                    if (v == v_prev) total_p += p;
                    else {
                        if (v_prev != -1) nxt_new.push_back({total_p, &vertices[v_prev]});
                        v_prev = v;
                        total_p = p;
                    }
                }
                if (!is_zero(total_p)) nxt_new.push_back({total_p, &vertices[v_prev]});
                u.nxt[ac] = nxt_new;

                // TEST: total p = 1
                if (u.nxt[ac].size() > 0) {
                    total_p = 0;
                    for (auto [p, v]: u.nxt[ac]) total_p += p;
                    assert(abs(total_p - 1) < 1e-9);
                }
            }
        }
    }

    void build(Vertex *v, function<double(int)> capture_prob, function<double(int)> flee_prob) {

        function<int(int)> level_add = [](int level) -> int {
            return min(level + 1, LEVEL_MAX);
        };
        function<int(int)> level_sub = [](int level) -> int {
            return max(level - 1, -LEVEL_MAX);
        };

        if (is_terminal(*v)) assert(false);
        int bc = v->ball_cnt;
        int cl = v->capture_level;
        int fl = v->flee_level;
        int r = v->round;
        // DONE: BAIT
        if (r+1 < round_max) {
            Vertex *new_state_bait_90 = &vertices[get_idx(bc, level_sub(cl), level_sub(fl), r+1)];
            double flee_rate_bait_90 = flee_prob(new_state_bait_90->flee_level);
            Vertex *new_state_bait_10 = &vertices[get_idx(bc, cl, level_sub(fl), r+1)];
            double flee_rate_bait_10 = flee_prob(new_state_bait_10->flee_level);
            add_edge(v, new_state_bait_90, 0.9 * (1 - flee_rate_bait_90), Action::BAIT);
            add_edge(v, fled, 0.9 * flee_rate_bait_90, Action::BAIT);
            add_edge(v, new_state_bait_10, 0.1 * (1 - flee_rate_bait_10), Action::BAIT);
            add_edge(v, fled, 0.1 * flee_rate_bait_10, Action::BAIT);
        }

        // DONE: MUD
        if (r+1 < round_max) {
            Vertex *new_state_mud_90 = &vertices[get_idx(bc, level_add(cl), level_add(fl), r+1)];
            double flee_rate_mud_90 = flee_prob(new_state_mud_90->flee_level);
            Vertex *new_state_mud_10 = &vertices[get_idx(bc, level_add(cl), fl, r+1)];
            double flee_rate_mud_10 = flee_prob(new_state_mud_10->flee_level);
            add_edge(v, new_state_mud_90, 0.9 * (1 - flee_rate_mud_90), Action::MUD);
            add_edge(v, fled, 0.9 * flee_rate_mud_90, Action::MUD);
            add_edge(v, new_state_mud_10, 0.1 * (1 - flee_rate_mud_10), Action::MUD);
            add_edge(v, fled, 0.1 * flee_rate_mud_10, Action::MUD);
        }

        // DONE: BALL
        double capture_rate = capture_prob(cl);
        add_edge(v, captured, capture_rate, Action::BALL);
        double flee_rate_capture = flee_prob(fl);
        add_edge(v, fled, (1 - capture_rate) * flee_rate_capture, Action::BALL);
        Vertex *not_fled = bc > 1 ? &vertices[get_idx(bc-1, cl, fl, 0)] : noball;
        add_edge(v, not_fled, (1 - capture_rate) * (1 - flee_rate_capture), Action::BALL);
    }

    void naive() {
        double p_ball = -1, p_bait = -1, p_mud = -1;
        function<double(Vertex*)> dfs = [&](Vertex *u) -> double {
            if (u->state == State::FLED) return 0;
            else if (u->state == State::CAPTURED) return 1;
            else if (u->state == State::NOBALL) return 0;

            // DONE: BALL
            double prob_ball = 0;
            for (auto [p, v]: u->nxt[(int)Action::BALL]) {
                prob_ball += p * dfs(v);
            }
            // DONE: BAIT
            double prob_bait = 0;
            for (auto [p, v]: u->nxt[(int)Action::BAIT]) {
                prob_bait += p * dfs(v);
            }
            // DONE: MUD
            double prob_mud = 0;
            for (auto [p, v]: u->nxt[(int)Action::MUD]) {
                prob_mud += p * dfs(v);
            }
            if (u == start) {
                p_ball = prob_ball;
                p_bait = prob_bait;
                p_mud = prob_mud;
            }
            return max({prob_ball, prob_bait, prob_mud});
        };
        double result = dfs(start);
        string decision;
        if (p_ball >= p_bait && p_ball >= p_mud) decision = decision_name(Action::BALL);
        else if (p_bait >= p_ball && p_bait >= p_mud) decision = decision_name(Action::BAIT);
        else if (p_mud >= p_ball && p_mud >= p_bait) decision = decision_name(Action::MUD);
        else assert(false);
        // cout << "The best decision is " << decision << " with probability " << result << endl;
        printf("Decision %s with probability %.3Lf%%\n", decision_name(Action::BALL).c_str(), p_ball*100);
        printf("Decision %s with probability %.3Lf%%\n", decision_name(Action::BAIT).c_str(), p_bait*100);
        printf("Decision %s with probability %.3Lf%%\n", decision_name(Action::MUD).c_str(), p_mud*100);
        printf("The best decision is %s with probability %.3Lf%%\n", decision.c_str(), result*100);
    }

    vector<vector<vector<vector<double>>>> dp;
    void dp_init() {
        int IBC = start->ball_cnt;
        dp.assign(IBC+1, vector<vector<vector<double>>>(LEVEL_ALL, vector<vector<double>>(LEVEL_ALL, vector<double>(round_max, 0))));
        for (int bc = 1; bc <= IBC; ++bc) {
            for (int r = round_max-1; r >= 0; --r) {
                for (int cl = -LEVEL_MAX; cl <= LEVEL_MAX; ++cl) {
                    for (int fl = -LEVEL_MAX; fl <= LEVEL_MAX; ++fl) {
                        if (r == round_max-1) dp[bc][cl+LEVEL_MAX][fl+LEVEL_MAX][r] = P_caught(cl) + (1-P_caught(cl))*(1-P_fled(fl))*dp[bc-1][cl+LEVEL_MAX][fl+LEVEL_MAX][0];
                        else {
                            double p_ball = P_caught(cl) + (1-P_caught(cl))*(1-P_fled(fl))*dp[bc-1][cl+LEVEL_MAX][fl+LEVEL_MAX][0];
                            double p_bait = 0.9 * (1.0-P_fled(max(fl-1, -LEVEL_MAX))) * dp[bc][max(cl-1, -LEVEL_MAX)+LEVEL_MAX][max(fl-1, -LEVEL_MAX)+LEVEL_MAX][r+1] + \
                                0.1 * (1.0-P_fled(max(fl-1, -LEVEL_MAX))) * dp[bc][cl+LEVEL_MAX][max(fl-1, -LEVEL_MAX)+LEVEL_MAX][r+1];
                            double p_mud = 0.9 * (1.0-P_fled(min(fl+1, LEVEL_MAX))) * dp[bc][min(cl+1, LEVEL_MAX)+LEVEL_MAX][min(fl+1, LEVEL_MAX)+LEVEL_MAX][r+1] + \
                                0.1*(1.0-P_fled(fl)) * dp[bc][min(cl+1, LEVEL_MAX)+LEVEL_MAX][fl+LEVEL_MAX][r+1];
                            dp[bc][cl+LEVEL_MAX][fl+LEVEL_MAX][r] = max({p_ball, p_bait, p_mud});
                        }
                    }
                }
            }
        }
    }

    vector<Vertex> print_decision(int bc, int r, int cl, int fl) {
        set<Vertex> s;
        printf("current state:\n");
        printf("  - ball count: %lld\n", bc);
        printf("  - round: %lld\n", r);
        printf("  - capture level: %lld\n", cl);
        printf("  - flee level: %lld\n", fl);
        printf("\n");
        if (r == round_max-1) {
            double p = P_caught(cl) + (1-P_caught(cl))*(1-P_fled(fl))*dp[bc-1][cl+LEVEL_MAX][fl+LEVEL_MAX][0];
            s.insert(Vertex(bc-1, cl, fl, 0));
            printf("Decision %s with probability %.3Lf%%\n", decision_name(Action::BALL).c_str(), p*100);
            printf("The best decision is %s with probability %.3Lf%%\n", decision_name(Action::BALL).c_str(), p*100);
        }
        else {
            double p_ball = P_caught(cl) + (1-P_caught(cl))*(1-P_fled(fl))*dp[bc-1][cl+LEVEL_MAX][fl+LEVEL_MAX][0];
            double p_bait = 0.9 * (1.0-P_fled(max(fl-1, -LEVEL_MAX))) * dp[bc][max(cl-1, -LEVEL_MAX)+LEVEL_MAX][max(fl-1, -LEVEL_MAX)+LEVEL_MAX][r+1] + \
                0.1 * (1.0-P_fled(max(fl-1, -LEVEL_MAX))) * dp[bc][cl+LEVEL_MAX][max(fl-1, -LEVEL_MAX)+LEVEL_MAX][r+1];
            double p_mud = 0.9 * (1.0-P_fled(min(fl+1, LEVEL_MAX))) * dp[bc][min(cl+1, LEVEL_MAX)+LEVEL_MAX][min(fl+1, LEVEL_MAX)+LEVEL_MAX][r+1] + \
                0.1*(1.0-P_fled(fl)) * dp[bc][min(cl+1, LEVEL_MAX)+LEVEL_MAX][fl+LEVEL_MAX][r+1];
            string decision;
            if (p_ball >= p_bait && p_ball >= p_mud) {
                s.insert(Vertex(bc-1, cl, fl, 0));
                decision = decision_name(Action::BALL);
            }
            else if (p_bait >= p_ball && p_bait >= p_mud) {
                s.insert(Vertex(bc, max(cl-1, -LEVEL_MAX), max(fl-1, -LEVEL_MAX), r+1));
                s.insert(Vertex(bc, cl, max(fl-1, -LEVEL_MAX), r+1));
                decision = decision_name(Action::BAIT);
            }
            else if (p_mud >= p_ball && p_mud >= p_bait) {
                s.insert(Vertex(bc, min(cl+1, LEVEL_MAX), min(fl+1, LEVEL_MAX), r+1));
                s.insert(Vertex(bc, min(cl+1, LEVEL_MAX), fl, r+1));
                decision = decision_name(Action::MUD);
            }
            else assert(false);
            printf("- Decision %s with probability %.3Lf%%\n", decision_name(Action::BALL).c_str(), p_ball*100);
            printf("- Decision %s with probability %.3Lf%%\n", decision_name(Action::BAIT).c_str(), p_bait*100);
            printf("- Decision %s with probability %.3Lf%%\n", decision_name(Action::MUD).c_str(), p_mud*100);
            printf("The best decision is %s with probability %.3Lf%%\n", decision.c_str(), dp[bc][r+LEVEL_MAX][cl+LEVEL_MAX][fl]*100);
        }
        vector<Vertex> vs = vector<Vertex>(s.begin(), s.end());
        return vs;
    }

    void dynamic_programming() {
        dp_init();
        int bc = start->ball_cnt, r = start->round, cl = start->capture_level, fl = start->flee_level;
        while (true) {
            if (bc == 0) break;
            auto s = print_decision(bc, r, cl, fl);
            if (s.size() == 0) break;
            printf("\n");
            printf("There are %lld outcomes of the best decision:\n", (int)s.size());
            for (int i = 0; i < (int)s.size(); ++i) {
                printf("%lld. ", i+1);
                printf("ball count: %lld, ", s[i].ball_cnt);
                printf("round: %lld, ", s[i].round);
                printf("capture level: %lld, ", s[i].capture_level);
                printf("flee level: %lld\n", s[i].flee_level);
            }
            string input;
            printf("Select the next state (1-%lld):\n", (int)s.size());
            cin >> input;
            int idx = stoi(input);
            bc = s[idx-1].ball_cnt;
            r = s[idx-1].round;
            cl = s[idx-1].capture_level;
            fl = s[idx-1].flee_level;
            printf("\n\n");
        }
    }

    void graphviz_dump(string filename) {
        ofstream fout(filename);
        fout << "digraph {\n";

        for (Vertex &u: vertices) {
            int idx_u = get_idx(u);
            if (u.state == State::FLED || u.state == State::CAPTURED || u.state == State::NOBALL) {
                if (u.state == State::FLED) fout << format("    v{}[shape=\"record\" label=\"v{}|FLED\"];\n", idx_u, idx_u);
                else if (u.state == State::CAPTURED) fout << format("    v{}[shape=\"record\" label=\"v{}|CAPTURED\"];\n", idx_u, idx_u);
                else if (u.state == State::NOBALL) fout << format("    v{}[shape=\"record\" label=\"v{}|NOBALL\"];\n", idx_u, idx_u);
                continue;
            }
            fout << format("    v{}[shape=\"record\" label=\"v{}|ball={}|cap={}|flee={}|round={}\"];\n", idx_u, idx_u, u.ball_cnt, u.capture_level, u.flee_level, u.round);
        }

        fout << "Legend [shape=none, margin=0, label=<\n";
            fout << "<TABLE BORDER=\"0\" CELLBORDER=\"1\" CELLSPACING=\"0\" CELLPADDING=\"4\">\n";
                fout << "<TR><TD COLSPAN=\"2\"><B>Legend</B></TD></TR>\n";
                fout << "<TR>\n";
                    fout << "<TD><FONT COLOR=\"red\">&#10148;</FONT></TD>\n";
                    fout << "<TD>throw ball</TD>\n";
                fout << "</TR>\n";
                fout << "<TR>\n";
                    fout << "<TD><FONT COLOR=\"green\">&#10148;</FONT></TD>\n";
                    fout << "<TD>throw bait</TD>\n";
                fout << "</TR>\n";
                fout << "<TR>\n";
                    fout << "<TD><FONT COLOR=\"blue\">&#10148;</FONT></TD>\n";
                    fout << "<TD>throw mud</TD>\n";
                fout << "</TR>\n";
            fout << "</TABLE>\n";
        fout << ">];\n";

        for (Vertex &u: vertices) {
            vector<pair<double, int>> equations;
            int idx_u = get_idx(u);
            for (auto [p, v]: u.nxt[(int)Action::BALL]) {
                int idx_v = get_idx(*v);
                fout << format("    v{} -> v{} [label=\"{:.4f}\", color=\"red\"];\n", idx_u, idx_v, p);
            }
            for (auto [p, v]: u.nxt[(int)Action::BAIT]) {
                int idx_v = get_idx(*v);
                fout << format("    v{} -> v{} [label=\"{:.4f}\", color=\"green\"];\n", idx_u, idx_v, p);
            }
            for (auto [p, v]: u.nxt[(int)Action::MUD]) {
                int idx_v = get_idx(*v);
                fout << format("    v{} -> v{} [label=\"{:.4f}\", color=\"blue\"];\n", idx_u, idx_v, p);
            }
        }

        fout << "}\n";
    }
};

struct Model {
    int base_capture_rate;
    int base_flee_rate;
    double safari_ball_rate;
    int initial_ball_cnt;
    string algorithm;
    int round_max;

    int round_floor(double x) {
        return (abs(x - round(x)) < 1e-9) ? round(x) : floor(x);
    }

    double capture_prob(int level) {
        assert(level >= -LEVEL_MAX and level <= LEVEL_MAX);
        double multiplier = level >= 0 ? (level+2) / 2.0 : 2.0 / (2-level);
        int a = round_floor((double)(1/3.0) * base_capture_rate * multiplier * safari_ball_rate);
        int G = round_floor(16711680.0 / a);
        G = round_floor(sqrt(G));
        G = round_floor(sqrt(G));
        G = round_floor(1048560.0 / G);
        double single = G / 65536.0;
        double prob = single * single * single * single;
        cmax(prob, (double)0.0);
        cmin(prob, (double)1.0);
        return prob;
    }

    double flee_prob(int level) {
        assert(level >= -LEVEL_MAX and level <= LEVEL_MAX);
        double multiplier = level >= 0 ? (level+2) / 2.0 : 2.0 / (2-level);
        double prob = (double)(base_flee_rate*multiplier + 1.0) / 255.0;
        cmax(prob, (double)0.0);
        cmin(prob, (double)1.0);
        return prob;
    }

    Model(int bcr, int bfr, double sbr, int ibc, string method, int rm) {
        base_capture_rate = bcr;
        base_flee_rate = bfr;
        safari_ball_rate = sbr;
        initial_ball_cnt = ibc;
        algorithm = method;
        round_max = rm;
    }

    void run() {
        if (algorithm == "lp") {
            Graph graph = Graph(initial_ball_cnt, [this](int level) -> double {return capture_prob(level);}, [this](int level) -> double {return flee_prob(level);});
        }
        else {
            GraphOpt graph = GraphOpt(initial_ball_cnt, round_max, [this](int level) -> double {return capture_prob(level);}, [this](int level) -> double {return flee_prob(level);});
            if (algorithm == "naive") graph.naive();
            else if (algorithm == "dp") graph.dynamic_programming();
            else assert(false);
        }
    }

};

int32_t main(int32_t argc, char **argv) {
    ios::sync_with_stdio(false); cin.tie(nullptr); cout.tie(nullptr);

    if (argc < 5 || argc > 7) {
        cerr << "Usage: " << argv[0] << " <base_capture_rate> <base_flee_rate> <safari_ball_rate> <initial_ball_cnt> [method]\n";
        cerr << "  methods:\n";
        cerr << "    naive [max_rounds=10]\n";
        cerr << "    dp [max_rounds=10]\n";
        cerr << "    lp\n";
        cerr << "  default: lp\n";
        return 1;
    }
    int bcr = stoi(argv[1]);
    int bfr = stoi(argv[2]);
    double sbr = stod(argv[3]);
    int ibc = stoi(argv[4]);
    string method = "lp";
    int round_max = 10;
    if (argc == 6 || argc == 7) {
        if (string(argv[5]) != "lp" && string(argv[5]) != "naive" && string(argv[5]) != "dp") {
            debug(string(argv[5]));
            cerr << "Invalid method\n";
            return 1;
        }
        method = argv[5];
        if (argc == 7) round_max = stoi(argv[6]);
    }

    Model model = Model(bcr, bfr, sbr, ibc, method, round_max);
    model.run();

    return 0;
}
