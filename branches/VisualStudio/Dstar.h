/* Dstar.h
 * James Neufeld (neufeld@cs.ualberta.ca)
 */

#ifndef DSTAR_H
#define DSTAR_H
#include <iostream>

#include <math.h>

#include <stack>

#include <queue>
#include <list>

#include <hash_map>


using namespace std;
//using namespace __gnu_cxx;

class state {
 public:
  int x;
  int y;
  pair<double,double> k;
  
  bool operator == (const state &s2) const {
    return ((x == s2.x) && (y == s2.y));
  }
  
  bool operator != (const state &s2) const {
    return ((x != s2.x) || (y != s2.y));
  }
  
  bool operator > (const state &s2) const {
    if (k.first-0.00001 > s2.k.first) return true;
    else if (k.first < s2.k.first-0.00001) return false;
    return k.second > s2.k.second;
  }

  bool operator <= (const state &s2) const {
    if (k.first < s2.k.first) return true;
    else if (k.first > s2.k.first) return false;
    return k.second < s2.k.second + 0.00001;
  }
  

  bool operator < (const state &s2) const {
    if (k.first + 0.000001 < s2.k.first) return true;
    else if (k.first - 0.000001 > s2.k.first) return false;
    return k.second < s2.k.second;
  }
   
};

struct ipoint2 {
  int x,y;
};

struct cellInfo {

  double g;
  double rhs;
  double cost;

};
struct greater_str {
   bool operator()(const state & x, const state & y) const {
      return x>y;
   }
};


struct equal_str {
   bool operator()(const state & x, const state & y) const {
      return x==y;
   }
};
class state_hash {
 public:
  size_t operator()(const state &s) const {
    return s.x + 34245*s.y;
  }
};
class statehashereq : public stdext::hash_compare <state>
{
public:
  /**
   * Required by 
   * Inspired by the java.lang.String.hashCode() algorithm 
   * (it's easy to understand, and somewhat processor cache-friendly)
   * @param The string to be hashed
   * @return The hash value of s
   */
  size_t operator() (const state& s) const
  {
    return s.x + 34245*s.y;
  }

  /**
   * 
   * @param s1 The first string
   * @param s2 The second string
   * @return true if the first string comes before the second in lexicographical order
   */
  bool operator() (const state& s1, const state& s2) const
  {
    return s1 != s2;
  }
};

typedef priority_queue<state, vector<state>, greater_str > ds_pq;
typedef stdext::hash_map <state,cellInfo,
statehashereq
> ds_ch;
typedef stdext::hash_map<state, float, 
statehashereq
> ds_oh;
//typedef hash_map<state,cellInfo, state_hash, equal_str > ds_ch;
//typedef hash_map<state, float, state_hash, equal_str > ds_oh;


class Dstar {
  
 public:
    
  Dstar();
  void   init(int sX, int sY, int gX, int gY);
  void   updateCell(int x, int y, double val);
  void   updateStart(int x, int y);
  void   updateGoal(int x, int y);
  bool   replan();
  void   draw();
  void   drawCell(state s,float z);

  list<state> getPath();
  
 private:
  
  list<state> path;

  double C1;
  double k_m;
  state s_start, s_goal, s_last;
  int maxSteps;  

  ds_pq openList;
  ds_ch cellHash;
  ds_oh openHash;

  bool   close(double x, double y);
  void   makeNewCell(state u);
  double getG(state u);
  double getRHS(state u);
  void   setG(state u, double g);
  void setRHS(state u, double rhs);
  double eightCondist(state a, state b);
  int    computeShortestPath();
  void   updateVertex(state u);
  void   insert(state u);
  void   remove(state u);
  double trueDist(state a, state b);
  double heuristic(state a, state b);
  state  calculateKey(state u);
  void   getSucc(state u, list<state> &s);
  void   getPred(state u, list<state> &s);
  double cost(state a, state b); 
  bool   occupied(state u);
  bool   isValid(state u);
  float  keyHashCode(state u);
};

#endif
