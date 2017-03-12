#ifndef __SensorNet__
#define __SensorNet__

#include <sstream>

#include "Graph.h"
#include "FifoQueue.h"

#define max(a, b) a > b ? a : b
#define min(a, b) a < b ? a : b

using std::stringstream;
using std::iostream;
using std::cout;
using std::cerr;
using std::pair;
/** Time **/
static int t = 0;

/** **/
static int c = 0;

/** logging string **/
static const char* ln =
  "--------------------------------------------------------------------------------\n";

double eElec = 100 * 10e-9;
double eAmp  = 100 * 10e-12;

/*****************************************************************************
 *
 ****************************************************************************/
/*struct SensorNode
{

}*/
typedef Vertex SensorNode;

/*****************************************************************************
 *
 ****************************************************************************/
struct SensorNet
{
  int xlen, //x:  field len meters
      ylen, //y:  field width meters
      area, //xy: field area meters
      N,    //N:  num nodes in network
      dgen, //p:  num data generators (DG)
      dval, //q:  num date generator items
      ncap, //m:  capacity of non DG nodes (400bytes)
      minR, //a:  min energy E range
      maxR, //b:  max energy E range
      e0;   //E0: initial energy E (rand val range [a, b]

  double txR;  //Tr: transmission range of a senseor node

  bool connected;

  Graph g;  //g:  Graph G = (V,E), where V is the set of sensors in network

  vector<SensorNode> sensors; //

  /**
   *
   */
  SensorNet()
  {
    //g.directed = true;
  };

  /**
   *
   */
  int InitE() { e0 = minR + (rand() % (maxR - minR + 1)); return e0; };

  /**
   *
   *
   */
  vector<SensorNode>& GetSensors() { return sensors; };

  double GetConsumption(Vertex & u, Vertex & v, double d)
  {
    double b = 0;
    double dd = d*d;
    double txp, rxp;
    if(u.dgen == true && v.dgen == false)
    {
      double dis = (u.di * 400) / v.m;
      b = dis * 8;
      txp = (eElec * b) + (eAmp * b * (dd));
      rxp = eElec * b;

    }
    else if(v.dgen == true && u.dgen == false)
    {
      double dis = (v.di * 400) / u.m;
      b = dis * 8;
      txp = (eElec * (v.m*8)) + (eAmp* (v.m*8) * (dd));
      rxp = eElec;
    }
    else if(v.dgen == true && u.dgen ==true)
    {
      return 0;
    }
    else
    {
      double b = min(u.m, v.m);
      b *= 8;
      txp = (eElec * b) + (eAmp * b * (dd));
      rxp = eElec * b;
    }
    return txp + rxp;
  }

  /**
   *
   */
  void GenerateNodes()
  {
    int px, py, dgs = 0;

    stringstream ss;

    for (int i = 0; i < N; ++i)
    {
      px = 1 + (rand() % (xlen-1));
      py = 1 + (rand() % (ylen-1));

      ss << i;
      SensorNode n(ss.str());
      cout << "Sensor[" << i << "]" << "x,y[" << px << ", " << py << "]" << endl;

      n.x = px;
      n.y = py;

      if(rand() % 2 == 0 && dgs < dgen)
      {
        n.dgen = true;
        n.di   = dval;
        n.m    = 0;
        dgs++;
      }
      else n.m = ncap;

      sensors.push_back(n);
      ss.str("");
      ss.clear();
    }

    vector<SensorNode>::iterator nit = sensors.begin();

    for ( size_t i = 0; nit != sensors.end(); ++nit, ++i)
    {
      vector<SensorNode>::iterator jit = sensors.begin();

      for(size_t j = 0;  j != i && jit != sensors.end(); ++jit, ++j)
      {
        double dx, dy, dist;

        dx   = sensors[j].x - sensors[i].x;
        dy   = sensors[j].y - sensors[i].y;
        dx  *= dx;
        dy  *= dy;

        dist = sqrt(dx + dy);

        if(dist < double(txR))
        {
          double w =  GetConsumption(sensors[i], sensors[j], dist);
          g.add_edge(sensors[i], sensors[j], w);
          cout << "connecting sensors " << i << "---" << j << endl;
          cout << "dist: " << sensors[i].id << ", " << sensors[j].id << endl
            << dx << ":" << dy << ":" << dist << ":" << txR << endl;
        }
        else
        {
          cout << "dist: " << sensors[i].id << ", " << sensors[j].id << " = " << dist
            << " > " << (double)txR << endl
            << dx << ":" << dy << ":" << dist << ":" << txR << endl;
        }
      }
    }

    cout << "Checking connectivity..." << endl;

    connected = CheckConnectivity();

    if(connected) cout << "Sensor network connection established";
  };

  /**
   * Explores adjacent verticies tracking discovery times to detect bccs
   * if a bridge is detected it will be added to a container
   *
   * @param Vertex* u is the vertex to visit
   * @param vector<Edge> is the collection of bccs
   */
  int BCC_VISIT(Vertex* & u, vector<Edge> & bccs, list < vector<Edge> >& abccs)
  {
    u->color    = eGray;
    u->d = u->l = ++t;
    Vertex* v   = 0;
    int minDu   = t, minDv, children = 0;

    cout << "BCC-VISIT: time:"
      << t << ", Visiting u: " << u->id << endl;

    for(AdjListIt uit = u->adj->begin(); uit != u->adj->end(); ++uit)
    {
      v = (*uit);
      cout << "BCC-VISIT: time:" << t << ", Exploring Adjacent Vertex"
        << " v: " << v->id << endl;

      if(v->color == eWhite)
      {
        children++;
        v->pi = &(*u);
        Edge edg(u, v);
        bccs.push_back(edg);
        minDv = BCC_VISIT(v, bccs, abccs);

        u->l = min(u->l, v->l);

        if( (u->d == 1 && children > 1) || (u->d > 1 && v->l >= u->d))
        {
          vector<Edge> ve;
          while(bccs.back().u != u || bccs.back().v != v)
          {
            ve.push_back(bccs.back());
            bccs.pop_back();
          }
          ve.push_back(bccs.back());
          bccs.pop_back();
          abccs.push_back(ve);
          c++;
        }
      }
      else if(v != u->pi && v->d < u->l)
      {
        minDu = v->d;
        u->l = min(u->l, v->d);
        Edge bridge(u, v);
        bccs.push_back(bridge);
      }
    }

    u->color = eBlack;
    u->f = ++t;
    cout << "BCC-VISIT: time:"
      << t << ", Finished u:" << u->id << endl << ln;

    return minDu;
  };

  /**
   * Finds bccs in a graph
   *
   * @param Graph G is the graph for which bccs should be detected
   * @return vector<Edge> is the collection of edges that are bccs
   */
  list < vector<Edge> > BICONNECTED_COMP()
  {
    vector<Edge> bccs;
    list< vector<Edge> > abccs;
    VertexMapIt vit = g.VE.begin();
    Vertex* u = 0;

    t = 0;
    for(; vit != g.VE.end(); ++vit)
    {
      u = (Vertex*)&vit->first;
      u->color  = eWhite;
      u->pi     = NIL;
      cout << "BCC      : time:" << t << ", "
           << "Initialized u:" << u->id << endl;
    }

    cout << ln;
    t = 0;

    for(vit = g.VE.begin(); vit != g.VE.end(); ++vit)
    {
      u = (Vertex*)&vit->first;
      cout << "BCC      : time:" << t << ", ";
      if(u->color == eWhite)
      {
        cout << "Visiting u:" << u->id << endl;
        BCC_VISIT(u, bccs, abccs);
      }
      else
        cout << "Not Visiting u:" <<  u->id << endl;

      size_t sz = bccs.size(), j = 0;
      vector<Edge> ev;
      while(bccs.size() > 0)
      {
        j = 1;
        ev.push_back(bccs.back());
        bccs.pop_back();
      }
      if(j)
      {
        abccs.push_back(ev);
        c++;
      }
    }

    cout << ln;

    return abccs;
  };

  /**
   *
   */
  bool CheckConnectivity()
  {
    bool rval = true;

    if(N <= g.VE.size())
    {
      list < vector<Edge> > abccs = BICONNECTED_COMP();
      VertexMapIt vit = g.VE.begin();
      Vertex* u = 0;

      for(vit = g.VE.begin(); vit != g.VE.end() && rval; ++vit)
      {
        u = (Vertex*)&vit->first;
        if(u->color != eBlack)
        {
          rval = false;
          cout << "Sensor: " << u->id << " was not visited " << endl;
        }
      }
    }
    else 
    {
      rval = false;
      cout << "All generated sensors were not within Tr" << endl;
      cout << "Total sensors connected: " << g.VE.size() << " < " << N/2 << endl;
    }

    return rval;
  };

  /**
   *
   */
  bool IsConnected() { return connected; }

  /**
   *
   */
  map< pair<string, string>, int> BELLMAN_FORD(Vertex* src, map< pair<string, string>, int> & pd)
  {
    for(VertexMapIt i = g.VE.begin(); i != g.VE.end(); ++i)
    {
      Vertex *v = (Vertex*) &i->first;
      if(*v == *src)
      {
        v->d = 0;
      }
      else
      {
        v->d = INT_MAX - 1000;
      }
      cout << v->id << " " << v->d << " " << v << endl;
      //v->p = NIL;
    }

    cout << "sz: " << g.esize() << endl;
    for(size_t j = 0; j < g.vsize(); ++j)
    {
      for(size_t k = 0; k < g.esize(); ++k)
      {
        Edge* e =  g.get_edge(k);
        Vertex* u = e->u, *v = e->v;
        g.relax(*u, *v, e->cap);
        cout << u->id << " " << u->d << " : "
          << v->id << " " << v->d << " " << v  << " " << u << endl;
      }

    }
    for(size_t k = 0; k < g.esize(); ++k)
    {
      Edge* e = g.get_edge(k);

      if(e)
      {
        if(e->u->d + e->cap < e->v->d)
        {
          cerr << "ERROR: Graph contains a negative weight cycle" << endl;
        }
      }
    }

    for(VertexMapIt i = g.VE.begin(); i != g.VE.end(); ++i)
    {
      Vertex *v = (Vertex*) &i->first;
      if(*v != *src && v->pi)
      {
        pair<string, string> k = make_pair(v->id, v->pi->id);
        pd[k] = v->d;
        //pd.insert(make_pair(v->id, v->pi->id), v->d);
      }
    }

    return pd;
  }

  void PRINT_APSP(map< pair<string, string>, int> & apsp)
  {
    cout << "Rendering APSP...\n";

    if(apsp.size() < 1)
    {
      cout << "None\n";
    }
    else
    {
      map< pair<string, string>, int>::iterator i = apsp.begin(),
        e = apsp.end();

      while(i != e)
      {
        cout << i->first.first << " -> " << i->first.second << " "
          << i->second << endl;
        ++i;
      }
    }
  }

  /**
   * Performs breadth first search on graph G = (V, E) with source vertex
   * s in V
   *
   * @param graph G is the graph to be searched
   * @param Vertex s is the source
   * @param Vertex t is the sink
   * @return int is the flow along path s->t
   */
  double BFS(Vertex & s, Vertex & t)
  {
    double r = 0;
    queue<Vertex> q;
    s.mcap = DBL_MAX;
    s.pi = &s;
    q.enqueue(s);

    while(!q.empty())
    {
      Vertex* u = (Vertex*) &g.VE.find(q.dequeue())->first;
      EdgePtrVector E = g.adjacent_edges(*u);

      for(size_t i = 0; i < E.size(); ++i) //for each edge adjacent to u
      {
        Edge* e   = E[i];
        Vertex* v = g.adjacent_vertex(*u, *e);

        if(v->pi == NIL && v->id != s.id && e->residual() != 0)
        //if(v->pi == NIL && v->id != s.id && e->residual())
        {
          v->pi   = (Vertex*) &(g.VE.find(*u)->first);
          v->mcap = min(u->mcap, e->residual());
          if(v->id != t.id)
          {
            cout << "queued: " << v->id << endl;
            q.enqueue(*v);
          }
          else
          {
            cout << "mcap: " << v->mcap << endl;
            return v->mcap;
          }
        }
      }
    }

    return r;
  }

  /**
   * Renders the resultant BFS path of EDMONDS_KARP
   */
  void print_path(map<Vertex, Vertex> & P)
  {
    stringstream ss;
    ss << "Rendering Path...\n";
    ss << P.begin()->second.id << " -> ";
    for(map<Vertex, Vertex>::iterator it = P.begin();
        it != P.end(); ++it)
    {
      Vertex v = it->first;
      ss << v.id;
      if(++it == P.end())
        ss << endl;
      else
        ss << " -> ";
      it--;
    }
    cout << ss.str();
  }

  /**
   * Implementation of the FordFulkerson method for computing the maximum flow
   * in a flow network in O(|V||E^2|)
   *
   * @param graph G is the specified graph
   * @param Vertex s is the source
   * @param Vertex t is the sink
   * @return int f is the max flow of G
   */
  double EDMONDS_KARP(Vertex s, Vertex t)
  {
    double m = DBL_MAX, f = 0;

    g.nilpi();
    while(m)
    {
      m = BFS(s, t);           //BFS returns capacity of flow for the path
      if(m == 0)  break;          //if 0 no additional flow can be sent
      cout << "f + m = " << f << " + " <<  m << " = " << f+m << endl;

      f += m;                     //update the flow
      Vertex v = *g.get_vertex(t);
      map<Vertex, Vertex> P;
      //if(v.id != s.id)
      //  P[s] = v;
      while(v.id != s.id)
      {
        Vertex u = *(v.pi);
        P[v] = u;
        g.update_edge(u, v, m);   //update edge and reverse edge flow
        //if(v.id == t.id) {m=0; break;}
        v = u;
      }
      print_path(P);
      g.nilpi();
    }

    cout << "EDMONDS_KARP: " << f << endl;
    return f;
  }

  /**
   *
   */
  double EnergyConsumption(string u, string v)
  {
    double rval = 0;

    VertexMapIt src = g.VE.find(Vertex(u));
    VertexMapIt snk = g.VE.find(Vertex(v));

    if(src == g.VE.end() || snk == g.VE.end())
      cout << "src snk not comnnected" << endl;
    else
    {
      Vertex * u = g.get_vertex(src->first);
      Vertex * v = g.get_vertex(snk->first);

      cout << "u: " << u->id << " v: " << v->id << endl;
      if(u->is_adj(v))
      {
        rval = GetConsumption(*u, *v, u->dist(*v));
        cout << u << " " << v << " adjacent" << endl;
      }
      else
      {
        g.directed = true;
        rval = EDMONDS_KARP(src->first, snk->first);
      }
    }
    return rval;
  };

  /**
   *
   *
   */
  int GetMinConnNodes()
  {
    int rval = 0;

    return rval;
  };

};

#endif//__SensorNet__
