/* Dstar.cpp
 * James Neufeld (neufeld@cs.ualberta.ca)
 */

#include "Dstar.h"
#include <stdio.h>

#ifdef USE_OPEN_GL
#ifdef MACOS
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

/* void Dstar::Dstar()
 * --------------------------
 * Constructor sets constants.
 */
Dstar::Dstar() { 

  maxSteps = 80000;  // node expansions before we give up
  C1       = 1;      // cost of an unseen cell

}

/* float Dstar::keyHashCode(state u) 
 * -------------------------- 
 * Returns the key hash code for the state u, this is used to compare
 * a state that have been updated
 */
float Dstar::keyHashCode(const state &u) const {

  return (float)(u.k.first + 1193*u.k.second);

}

/* bool Dstar::isValid(state u) 
 * --------------------------
 * Returns true if state u is on the open list or not by checking if
 * it is in the hash table.
 */
bool Dstar::isValid(const state &u) const {
  
  ds_oh::const_iterator cur = openHash.find(u);
  if (cur == openHash.end()) return false;
  if (!near(keyHashCode(u), cur->second)) return false;
  return true;
  
}

/* void Dstar::getPath() 
 * --------------------------
 * Returns the path created by replan()
 */
ds_path &Dstar::getPath() {
  return path;
}

/* bool Dstar::occupied(state u)
 * --------------------------
 * returns true if the cell is occupied (non-traversable), false
 * otherwise. non-traversable are marked with a cost < 0.
 */
bool Dstar::occupied(const state &u) const{
  
  ds_ch::const_iterator cur = cellHash.find(u);
  
  if (cur == cellHash.end()) return false;
  return (cur->second.cost < 0);
}

/* void Dstar::init(int sX, int sY, int gX, int gY)
 * --------------------------
 * Init dstar with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void Dstar::init(int sX, int sY, int gX, int gY) {
  
  cellHash.clear();
  path.clear();
  openHash.clear();
  while(!openList.empty()) openList.pop();

  k_m = 0;
  
  s_start.x = sX;
  s_start.y = sY;
  s_goal.x  = gX;
  s_goal.y  = gY;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  calculateKey(s_start);

  s_last = s_start;

}
/* void Dstar::makeNewCell(state u)
 * --------------------------
 * Checks if a cell is in the hash table, if not it adds it in.
 */
void Dstar::makeNewCell(const state &u) {
  
  if (cellHash.find(u) != cellHash.end()) return;

  cellInfo tmp;
  tmp.g       = tmp.rhs = heuristic(u,s_goal);
  tmp.cost    = C1;
  cellHash[u] = tmp;
  
}

/* double Dstar::getG(state u)
 * --------------------------
 * Returns the G value for state u.
 */
double Dstar::getG(const state &u) const {

  ds_ch::const_iterator cur = cellHash.find(u);
  if ( cur == cellHash.end() ) 
    return heuristic(u,s_goal);
  return cur->second.g;
  //return cellHash[u].g;
  
}

/* double Dstar::getRHS(state u)
 * --------------------------
 * Returns the rhs value for state u.
 */
double Dstar::getRHS(const state &u) const {

  if (u == s_goal) return 0;  

  ds_ch::const_iterator cur = cellHash.find(u);
  if (cur == cellHash.end()) 
    return heuristic(u,s_goal);
  return cur->second.rhs;
  //return cellHash[u].rhs;
  
}

/* void Dstar::setG(state u, double g)
 * --------------------------
 * Sets the G value for state u
 */
void Dstar::setG(const state &u, double g) {
  
  makeNewCell(u);  
  cellHash[u].g = g; 
}

/* void Dstar::setRHS(state u, double rhs)
 * --------------------------
 * Sets the rhs value for state u
 */
void Dstar::setRHS(const state &u, double rhs) {
  
  makeNewCell(u);
  cellHash[u].rhs = rhs;

}

/* double Dstar::eightCondist(state a, state b) 
 * --------------------------
 * Returns the 8-way distance between state a and state b.
 */
double Dstar::eightCondist(const state &a, const state &b) const {
  double temp;
  double min = abs(a.x - b.x);
  double max = abs(a.y - b.y);
  if (min > max) {
    temp = min;
    min = max;
    max = temp;
  }
  return ((M_SQRT2-1.0)*min + max);
}


/** \brief Check that a node is consistent */
bool Dstar::isConsistent(const state &u) {
  return (getRHS(u) == getG(u));
}

/* int Dstar::computeShortestPath()
 * --------------------------
 * As per [S. Koenig, 2002] except for 2 main modifications:
 * 1. We stop planning after a number of steps, 'maxsteps' we do this
 *    because this algorithm can plan forever if the start is
 *    surrounded by obstacles. 
 * 2. We lazily remove states from the open list so we never have to
 *    iterate through it.
 */
int Dstar::computeShortestPath() {
  
  list<state> s;
  list<state>::iterator i;

  if (openList.empty()) return 1;

  int k=0;
  while ((!openList.empty()) && 
         (openList.top() < (calculateKey(s_start))) || 
	 (!isConsistent(s_start))) {
    //(getRHS(s_start) != getG(s_start))) {

    if (k++ > maxSteps) {
      fprintf(stderr, "At maxsteps\n");
      return -1;
    }


    state u;

    // check consistency (one of the loop conditions)
    bool test = isConsistent(s_start);
    //(getRHS(s_start) != getG(s_start));
    
    // lazy remove
    while(1) { 
      if (openList.empty()) return 1; // checks outer loop condition #1
      u = openList.top();
      openList.pop();
      
      if (!isValid(u)) continue;
      if (!(u < s_start) && test){
	return 2; // checks outer loop conditions #2,3 still hold
      }
      break;
    }
    
    // At this point, the top-most valid state is popped
    ds_oh::iterator cur = openHash.find(u);
    openHash.erase(cur);

    state k_old = u;

    if (k_old < calculateKey(u)) { // u is out of date
      insert(u); // u has been removed already, reinsert into pq with new key
    } else if (getG(u) > getRHS(u)) { // needs update (got better)
      setG(u,getRHS(u));
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
    } else {   // g <= rhs, state has got worse
      setG(u,INFINITY);
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
      updateVertex(u);
    }
  }
  return 0;
}

/* bool Dstar::near(double x, double y) 
 * --------------------------
 * Returns true if x and y are within 10e-10, false otherwise
 */
bool Dstar::near(double x, double y) const {
    
  if (isinf(x) && isinf(y)) return true;
  return (fabs(x-y) < 10e-10);
  
}

/* void Dstar::updateVertex(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateVertex(state &u) {

  list<state> s;
  list<state>::iterator i;
 
  if (u != s_goal) {
    getSucc(u,s);
    double tmp = INFINITY;
    double tmp2;

    for (i=s.begin();i != s.end(); i++) {
      tmp2 = getG(*i) + cost(u,*i);
      if (tmp2 < tmp) tmp = tmp2;
    }
    if (!near(getRHS(u),tmp)) setRHS(u,tmp);
  }

  // remove u from the hashList if it is there
  // (it will be lazily removed from the openList later)
  remove(u);
  
  if (!near(getG(u),getRHS(u))) insert(u);
  
}


/* void Dstar::insert(state u) 
 * --------------------------
 * Inserts state u into openList and openHash.
 */
void Dstar::insert(state &u) {
  
  ds_oh::iterator cur;
  float csum;

  calculateKey(u);
  cur  = openHash.find(u);
  csum = keyHashCode(u);  //TODO: check if unique hash code here (check state as well)

  // return if cell is already in list. TODO: this should be
  // uncommented except it introduces a bug, I suspect that there is a
  // bug somewhere else and having duplicates in the openList queue
  // hides the problem...
  if ((cur != openHash.end()) && (near(csum,cur->second))) return;

  openHash[state(u)] = csum;
  openList.push(state(u));
} 

/* void Dstar::remove(state u)
 * --------------------------
 * Removes state u from openHash. The state is removed from the
 * openList lazily (in computeShortestPath()) to save computation.
 */
void Dstar::remove(const state &u) {
  
  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return;
  openHash.erase(cur);

}


/* double Dstar::trueDist(state a, state b) 
 * --------------------------
 * Euclidean cost between state a and state b.
 */
double Dstar::trueDist(const state &a, const state &b) const {
  
  float x = a.x-b.x;
  float y = a.y-b.y;
  return sqrt(x*x + y*y);
  
}

/* double Dstar::heuristic(state a, state b)
 * --------------------------
 * Pretty self explanitory, the heristic we use is the 8-way distance
 * scaled by a constant C1 (should be set to <= min cost).
 */
double Dstar::heuristic(const state &a, const state &b) const {
  return eightCondist(a,b)*C1;
}

/* state Dstar::calculateKey(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
state &Dstar::calculateKey(state &u) const {
  
  double val = fmin(getRHS(u),getG(u));
  
  u.k.first  = val + heuristic(u,s_start) + k_m;
  u.k.second = val;

  return u;

}

/* double Dstar::cost(state a, state b)
 * --------------------------
 * Returns the cost of moving from state a to state b. This could be
 * either the cost of moving off state a or onto state b, we went with
 * the former. This is also the 8-way cost.
 */
double Dstar::cost(const state &a, const state &b) const {

  int xd = abs(a.x-b.x);
  int yd = abs(a.y-b.y);
  double scale = 1;

  if (xd+yd>1) scale = M_SQRT2;

  ds_ch::const_iterator cur = cellHash.find(a);
  if( cur == cellHash.end() ) return scale*C1;
  return scale*(cur->second).cost;

  //if (cellHash.count(a) == 0) return scale*C1;
  //return scale*cellHash[a].cost;

}

/* void Dstar::updateCell(int x, int y, double val)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateCell(int x, int y, double val) {
  
  state u;
  
  u.x = x;
  u.y = y;

  if ((u == s_start) || (u == s_goal)) return;

  // if the value is still the same, don't need to do anything
  if( near(cellHash[u].cost, val) )  return;

  makeNewCell(u);
  cellHash[u].cost = val;

  updateVertex(u);
}

/* void Dstar::getSucc(state u,list<state> &s)
 * --------------------------
 * Returns a list of successor states for state u, since this is an
 * 8-way graph this list contains all of a cells neighbours. Unless
 * the cell is occupied in which case it has no successors. 
 */
void Dstar::getSucc(state u,list<state> &s) const {

  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  if (occupied(u)) return;

  u.x += 1;
  s.push_front(u);
  u.y += 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);

}

/* void Dstar::getPred(state u,list<state> &s)
 * --------------------------
 * Returns a list of all the predecessor states for state u. Since
 * this is for an 8-way connected graph the list contails all the
 * neighbours for state u. Occupied neighbours are not added to the
 * list.
 */
void Dstar::getPred(state u,list<state> &s) const {
  
  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.y += 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  
}

/* void Dstar::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void Dstar::updateStart(int x, int y) {

  s_start.x = x;
  s_start.y = y;
  
  k_m += heuristic(s_last,s_start);

  s_start = calculateKey(s_start);
  s_last  = s_start;
  
}

/* void Dstar::updateGoal(int x, int y)
 * --------------------------
 * This is somewhat of a hack, to change the position of the goal we
 * first save all of the non-empty on the map, clear the map, move the
 * goal, and re-add all of non-empty cells. Since most of these cells
 * are not between the start and goal this does not seem to hurt
 * performance too much. Also it free's up a good deal of memory we
 * likely no longer use.
 */
void Dstar::updateGoal(int x, int y) {
   
  list< pair<ipoint2, double> > toAdd;
  pair<ipoint2, double> tp;
  
  ds_ch::iterator i;
  list< pair<ipoint2, double> >::iterator kk;
  
  for(i=cellHash.begin(); i!=cellHash.end(); i++) {
    if (!near(i->second.cost, C1)) {
      tp.first.x = i->first.x;
      tp.first.y = i->first.y;
      tp.second = i->second.cost;
      toAdd.push_back(tp);
    }
  }

  cellHash.clear();
  openHash.clear();

  while(!openList.empty())
    openList.pop();
  
  k_m = 0;
  
  s_goal.x  = x;
  s_goal.y  = y;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;    

  for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
    updateCell(kk->first.x, kk->first.y, kk->second);
  }
  

}

/* bool Dstar::replan()
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 *  the metric euclidean(state, goal) + euclidean(state,start). 
 */
bool Dstar::replan() {

  path.clear(); 

  int res = computeShortestPath();
  //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    path.cost = INFINITY;
    return false;
  }
  list<state> n;
  list<state>::iterator i;

  state cur = s_start; 
  state prev = s_start;

  if (isinf(getG(s_start))) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    path.cost = INFINITY;
    return false;
  }
  
  // constructs the path
  while(cur != s_goal) {
    
    path.path.push_back(cur);
    path.cost += cost(prev,cur);
    getSucc(cur, n);

    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL\n");
      path.cost = INFINITY;
      return false;
    }

    double cmin = INFINITY;
    double tmin = INFINITY;
    state smin = cur;
    
    for (i=n.begin(); i!=n.end(); i++) {
  
      //if (occupied(*i)) continue;
      double val  = cost(cur,*i);
      double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
      val += getG(*i);

      if (near(val,cmin)) {
        if (tmin > val2) {
          tmin = val2;
          cmin = val;
          smin = *i;
        }
      } else if (val < cmin) {
        tmin = val2;
        cmin = val;
        smin = *i;
      }
    }
    n.clear();
    prev = cur;
    cur = smin;
  }
  path.path.push_back(s_goal);
  path.cost += cost(prev,s_goal);
  return true;
}

#ifdef USE_OPEN_GL

void Dstar::draw() const {

  ds_ch::const_iterator iter;
  ds_oh::const_iterator iter1;
  state t;

  list<state>::const_iterator iter2;
  
  glBegin(GL_QUADS);
  for(iter=cellHash.begin(); iter != cellHash.end(); iter++) {
    if (iter->second.cost == 1) glColor3f(0,1,0);
    else if (iter->second.cost < 0 ) glColor3f(1,0,0);
    else glColor3f(0,0,1);
    drawCell(iter->first,0.45);
  }

  glColor3f(1,1,0);
  drawCell(s_start,0.45);
  glColor3f(1,0,1);
  drawCell(s_goal,0.45);

  for(iter1=openHash.begin(); iter1 != openHash.end(); iter1++) {
    glColor3f(0.4,0,0.8);
    drawCell(iter1->first, 0.2);
  }

  
  glEnd();

  glLineWidth(4);
  glBegin(GL_LINE_STRIP);
  glColor3f(0.6, 0.1, 0.4);

  for(iter2=path.path.begin(); iter2 != path.path.end(); iter2++) {
    glVertex3f(iter2->x, iter2->y, 0.2);
  }
  glEnd();

}

void Dstar::drawCell(const state &s, float size) const {

  float x = s.x;
  float y = s.y;
  
  
  glVertex2f(x - size, y - size);
  glVertex2f(x + size, y - size);
  glVertex2f(x + size, y + size);
  glVertex2f(x - size, y + size);


}

#else
void Dstar::draw() const {}
void Dstar::drawCell(const state &s, float z) const {}
#endif
