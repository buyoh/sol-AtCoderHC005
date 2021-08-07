#pragma GCC optimize("O3")
#include <bits/stdc++.h>

using namespace std;
using ll = long long int;

#define all(v) (v).begin(), (v).end()
#define repeat(cnt, l)                                                         \
  for (typename remove_const<                                                  \
           typename remove_reference<decltype(l)>::type>::type cnt = {};       \
       (cnt) < (l); ++(cnt))
#define rrepeat(cnt, l) for (auto cnt = (l)-1; 0 <= (cnt); --(cnt))
#define iterate(cnt, b, e) for (auto cnt = (b); (cnt) != (e); ++(cnt))
#define diterate(cnt, b, e) for (auto cnt = (b); (cnt) != (e); --(cnt))
const long long MD = 1000000007ll;
const long double PI = 3.1415926535897932384626433832795L;
template <typename T1, typename T2>
inline ostream &operator<<(ostream &o, const pair<T1, T2> p) {
  o << '(' << p.first << ':' << p.second << ')';
  return o;
}
template <typename T> inline T &chmax(T &to, const T &val) {
  return to = max(to, val);
}
template <typename T> inline T &chmin(T &to, const T &val) {
  return to = min(to, val);
}
void bye(string s, int code = 0) {
  cout << s << endl;
  exit(code);
}
struct XorShift {
  using result_type = uint64_t;
  result_type x_;
  XorShift(result_type x = 88172645463325252ull) : x_(x){};
  static constexpr inline result_type min() { return 0ull; }
  static constexpr inline result_type max() {
    return numeric_limits<result_type>::max();
  }
  inline result_type operator()() {
    x_ ^= x_ << 7;
    return x_ ^= x_ >> 9;
  }
  inline void discard(unsigned long long z) {
    while (z--)
      operator()();
  }
};
XorShift randdev;
template <typename T, typename Random = decltype(randdev),
          typename enable_if<is_integral<T>::value>::type * = nullptr>
inline T rand(T l, T h, Random &rand = randdev) {
  return uniform_int_distribution<T>(l, h)(rand);
}
template <typename T, typename Random = decltype(randdev),
          typename enable_if<is_floating_point<T>::value>::type * = nullptr>
inline T rand(T l, T h, Random &rand = randdev) {
  return uniform_real_distribution<T>(l, h)(rand);
}
template <typename T>
static ostream &operator<<(ostream &o, const std::vector<T> &v) {
  o << "[ ";
  for (const auto &e : v)
    o << e << ' ';
  return o << ']';
}

template <typename I> struct MyRangeFormat {
  I b, e;
  MyRangeFormat(I _b, I _e) : b(_b), e(_e) {}
};
template <typename I>
static ostream &operator<<(ostream &o, const MyRangeFormat<I> &f) {
  o << "[ ";
  iterate(i, f.b, f.e) o << *i << ' ';
  return o << ']';
}
template <typename I> struct MyMatrixFormat {
  const I &p;
  long long n, m;
  MyMatrixFormat(const I &_p, long long _n, long long _m)
      : p(_p), n(_n), m(_m) {}
};
template <typename I>
static ostream &operator<<(ostream &o, const MyMatrixFormat<I> &f) {
  o << '\n';
  repeat(i, (f.n)) {
    repeat(j, f.m) o << f.p[i][j] << ' ';
    o << '\n';
  }
  return o;
}
struct LOG_t {
  ~LOG_t() { cout << endl; }
};
#define LOG (LOG_t(), cout << 'L' << __LINE__ << ": ")
#define FMTA(m, w) (MyRangeFormat<decltype(m + 0)>(m, m + w))
#define FMTR(b, e) (MyRangeFormat<decltype(e)>(b, e))
#define FMTV(v) FMTR(v.begin(), v.end())
#define FMTM(m, h, w) (MyMatrixFormat<decltype(m + 0)>(m, h, w))

#if defined(_WIN32) || defined(_WIN64)
#define getc_x _getc_nolock
#define putc_x _putc_nolock
#elif defined(__GNUC__)
#define getc_x getc_unlocked
#define putc_x putc_unlocked
#else
#define getc_x getc
#define putc_x putc
#endif
class MaiScanner {
  FILE *fp_;
  constexpr bool isvisiblechar(char c) noexcept {
    return (0x21 <= (c) && (c) <= 0x7E);
  }

public:
  inline MaiScanner(FILE *fp) : fp_(fp) {}
  template <typename T> void input_integer(T &var) noexcept {
    var = 0;
    T sign = 1;
    int cc = getc_x(fp_);
    for (; cc < '0' || '9' < cc; cc = getc_x(fp_))
      if (cc == '-')
        sign = -1;
    for (; '0' <= cc && cc <= '9'; cc = getc_x(fp_))
      var = (var << 3) + (var << 1) + cc - '0';
    var = var * sign;
  }
  inline int c() noexcept { return getc_x(fp_); }
  template <typename T, typename enable_if<is_integral<T>::value,
                                           nullptr_t>::type = nullptr>
  inline MaiScanner &operator>>(T &var) noexcept {
    input_integer<T>(var);
    return *this;
  }
  inline MaiScanner &operator>>(string &var) {
    int cc = getc_x(fp_);
    for (; !isvisiblechar(cc); cc = getc_x(fp_))
      ;
    for (; isvisiblechar(cc); cc = getc_x(fp_))
      var.push_back(cc);
    return *this;
  }
  template <typename IT> inline void in(IT begin, IT end) {
    for (auto it = begin; it != end; ++it)
      *this >> *it;
  }
};
class MaiPrinter {
  FILE *fp_;

public:
  inline MaiPrinter(FILE *fp) : fp_(fp) {}
  template <typename T> void output_integer(T var) noexcept {
    if (var == 0) {
      putc_x('0', fp_);
      return;
    }
    if (var < 0)
      putc_x('-', fp_), var = -var;
    char stack[32];
    int stack_p = 0;
    while (var)
      stack[stack_p++] = '0' + (var % 10), var /= 10;
    while (stack_p)
      putc_x(stack[--stack_p], fp_);
  }
  inline MaiPrinter &operator<<(char c) noexcept {
    putc_x(c, fp_);
    return *this;
  }
  template <typename T, typename enable_if<is_integral<T>::value,
                                           nullptr_t>::type = nullptr>
  inline MaiPrinter &operator<<(T var) noexcept {
    output_integer<T>(var);
    return *this;
  }
  inline MaiPrinter &operator<<(char *str_p) noexcept {
    while (*str_p)
      putc_x(*(str_p++), fp_);
    return *this;
  }
  inline MaiPrinter &operator<<(const string &str) {
    const char *p = str.c_str();
    const char *l = p + str.size();
    while (p < l)
      putc_x(*p++, fp_);
    return *this;
  }
  template <typename IT> void join(IT begin, IT end, char sep = ' ') {
    for (bool b = 0; begin != end; ++begin, b = 1)
      b ? *this << sep << *begin : *this << *begin;
  }
};
MaiScanner scanner(stdin);
MaiPrinter printer(stdout);

struct P {
  using T = int;
  T y, x;

  inline explicit P(T _y, T _x) : y(_y), x(_x) {}
  inline P() : y(0), x(0) {}

  inline bool operator==(P p) const { return y == p.y && x == p.x; }
  inline bool operator!=(P p) const { return !operator==(p); }
  inline bool operator<(P p) const { return y == p.y ? x < p.x : y < p.y; }
  inline P operator+(P p) const { return P(y + p.y, x + p.x); }
  inline P operator-(P p) const { return P(y - p.y, x - p.x); }
  inline P &operator+=(P p) {
    y += p.y;
    x += p.x;
    return *this;
  }
  inline P &operator-=(P p) {
    y -= p.y;
    x -= p.x;
    return *this;
  }
  inline P &operator*=(T m) {
    y *= m;
    x *= m;
    return *this;
  }
  inline T distM(P p) const { return abs(y - p.y) + abs(x - p.x); }
  inline T distC(P p) const { return max(abs(y - p.y), abs(x - p.x)); }
  template <typename ITR> ITR nearestM(ITR begin, ITR end) const {
    if (begin == end)
      return end;
    T best = distM(*begin);
    ITR besti = begin;
    for (ITR it = begin; ++it, it != end;) {
      T m = distM(*it);
      if (best < m) {
        best = m;
        besti = it;
      }
    }
    return besti;
  }
};
inline ostream &operator<<(ostream &os, P p) {
  os << '(' << p.y << ',' << p.x << ')';
  return os;
}

const P FourMoving[] = {P(-1, 0), P(0, 1), P(1, 0), P(0, -1)};
const P FiveMoving[] = {P(-1, 0), P(0, 1), P(1, 0), P(0, -1), P(0, 0)};
const P EightMoving[] = {P(-1, 0),  P(0, 1),  P(1, 0),  P(0, -1),
                         P(-1, -1), P(-1, 1), P(1, -1), P(1, 1)};
enum {
  kUP = 0,
  kRIGHT = 1,
  kDOWN = 2,
  kLEFT = 3,
  kDIRNEUTRAL = 4,
};

inline P operator*(P::T m, P p) noexcept { return P(m * p.y, m * p.x); }

template <typename T>
// using T = int;
struct F {
  int height, width;
  vector<T> data;

  F(int h = 1, int w = 1) : height(h), width(w), data(h * w) {}

  inline T &operator()(int y, int x) { return data[x + y * width]; }
  inline T &operator()(P p) { return data[p.x + p.y * width]; }
  inline T operator()(int y, int x) const { return data[x + y * width]; }
  inline T operator()(P p) const { return data[p.x + p.y * width]; }

  inline bool safe(int y, int x) const {
    return 0 <= y && y < height && 0 <= x && x < width;
  }
  inline bool safe(P p) const {
    return 0 <= p.y && p.y < height && 0 <= p.x && p.x < width;
  }

  inline void fill(T e) { std::fill(data.begin(), data.end(), e); }
  inline void resize(int h, int w) {
    height = h;
    width = w;
    data.resize(h * w);
  }

  void print(ostream &os, int setw_arg = 4) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x)
        os << setw(setw_arg) << operator()(y, x) << ' ';
      os << '\n';
    }
  }
};

template <typename C = std::chrono::milliseconds> class Timer {
  std::chrono::system_clock::time_point tp_;

public:
  static inline auto now() { return std::chrono::system_clock::now(); }
  inline void tic() { tp_ = now(); }
  inline auto toc() const {
    return std::chrono::duration_cast<C>(now() - tp_).count();
  }
  inline Timer() : tp_(now()) {}
};
inline std::ostream &operator<<(std::ostream &o, const Timer<> &t) {
  return o << (long long)t.toc();
}

// template <typename G> void warshall_floyd(G &g) {
//   int i, j, k;
//   for (i = 0; i < g.n; i++) {
//     for (j = 0; j < g.n; j++) {
//       for (k = 0; k < g.n; k++) {
//         g(j, k) = std::min(g(j, k), g(j, i) + g(i, k));
//       }
//     }
//   }
// }

//

constexpr int inf = numeric_limits<int>::max() / 4;
int N;
P kStartPos;
F<int> field;
Timer progTimer;

//

namespace solver {

struct PreCalcSt {
  int dist;
  int dir;
  PreCalcSt() : dist(inf), dir(0){};
  PreCalcSt(int _dist, int _dir) : dist(_dist), dir(_dir){};
};

struct PreCalc {
  F<PreCalcSt> st;
  vector<P> sight;
};

F<PreCalc> preCalc;

//

F<PreCalcSt> initPreCalcStField(P start) {
  priority_queue<pair<ll, pair<P, int>>> pque;

  F<PreCalcSt> dist(N, N);
  dist(start) = PreCalcSt{0, 0};
  repeat(vi, 4) {
    auto v = FourMoving[vi];
    auto p = start + v;
    if (!field.safe(p) || field(p) == inf) {
      continue;
    }
    pque.emplace(-1, pair<P, int>(p, vi));
    dist(p) = PreCalcSt{1, vi};
  }

  while (!pque.empty()) {
    auto pp = pque.top();
    pque.pop();
    auto d = -pp.first;
    auto p = pp.second.first;
    auto firstDir = pp.second.second;

    for (auto p2 : FourMoving) {
      p2 += p;
      if (!dist.safe(p2))
        continue;
      auto d2 = d + field(p2);
      if (dist(p2).dist > d2) {
        dist(p2).dist = d2;
        dist(p2).dir = firstDir;
        pque.emplace(-d2, pair<P, int>(p2, firstDir));
      }
    }
  }
  return dist;
}

vector<P> calcSight(P pos) {
  vector<P> sight;
  for (P v : FourMoving) {
    for (P p = pos + v; field.safe(p) && field(p) != inf; p += v)
      sight.push_back(p);
  }
  sight.shrink_to_fit();
  return sight;
}

void initPreCalc() {
  preCalc.resize(N, N);
  repeat(y1, N) {
    repeat(x1, N) {
      if (field(y1, x1) == inf)
        continue;
      preCalc(y1, x1).st = initPreCalcStField(P{y1, x1});
      preCalc(y1, x1).sight = calcSight(P{y1, x1});
    }
  }
}

double calcScoreFromTime(ll time) { return double(N) * 100000.0 / time; }

pair<double, vector<int>> solveSingle() {
  Timer timer;

  // TODO: 戦略
  // - 外側へ誘導する？？
  // - 初手で外側を選び、そこへ誘導する。 不要な末端への移動になりがちな点は
  //   注意。これは、目的地がvisitedになったかどうかを確認すればおｋ

  F<char> visited(N, N);
  repeat(y, N) {
    repeat(x, N) {
      bool wall = field(y, x) == inf;
      visited(y, x) = wall;
    }
  }

  vector<int> commands;
  ll scoreTime = 0;

  P pos = kStartPos;
  // fill sight
  visited(pos) = true;
  for (auto p : preCalc(pos).sight)
    visited(p) = true;

  // fill non-visited cell
  while (true) {
    // search for next moving
    pair<int, int> best = {inf, -1};
    repeat(y, N) {
      repeat(x, N) {
        if (visited(y, x))
          continue;
        const auto &dd = preCalc(pos).st(y, x);
        int score = dd.dist + rand(0, 50) + x;
        chmin(best, make_pair(score, dd.dir));
      }
    }
    if (best.second == -1)
      break;

    // move
    commands.emplace_back(best.second);
    pos += FourMoving[best.second];
    scoreTime += field(pos);

    // fill sight
    visited(pos) = true;
    for (auto p : preCalc(pos).sight)
      visited(p) = true;
  }
  //
  while (pos != kStartPos) {
    const auto &dd = preCalc(pos).st(kStartPos);
    // move
    commands.emplace_back(dd.dir);
    pos += FourMoving[dd.dir];
    scoreTime += field(pos);
  }

  // clog << "time: " << timer.toc() << endl;
  return make_pair(calcScoreFromTime(scoreTime), std::move(commands));
  //
}

void solve() {

  initPreCalc();

  //

  pair<double, vector<int>> best;
  best.first = 0;
  int loopcount = 0;
  while (progTimer.toc() < 2980) {
    ++loopcount;
    auto res = solveSingle();
    if (best.second.empty())
      clog << "first score:" << res.first << endl;
    if (best.first < res.first) {
      swap(best, res);
    }
  }

  for (auto c : best.second) {
    printer << "URDL"[c];
  }
  printer << '\n';

  clog << "final score: " << best.first << " loopcount: " << loopcount << endl;
  clog << int(best.first * 100) << endl;
}

//

} // namespace solver

//

void input() {
  int n, si, sj;
  scanner >> n >> si >> sj;
  N = n;
  kStartPos = P{si, sj};
  field = decltype(field)(N, N);
  repeat(i, N) {
    string str;
    scanner >> str;
    repeat(j, N) {
      char c = str[j];
      field(i, j) = c == '#' ? inf : (c - '0');
    }
  }
}

int main() {

  input();
  solver::solve();

  return 0;
}
