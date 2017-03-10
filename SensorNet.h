#ifndef __SensorNet__
#define __SensorNet__

#include <sstream>

#include "Graph.h"

#define max(a, b) a > b ? a : b
#define min(a, b) a < b ? a : b

using std::stringstream;
using std::iostream;
using std::cout;
using std::cerr;

/** Time **/
static int t = 0;

/** **/
static int c = 0;

/** logging string **/
static const char* ln =
  "--------------------------------------------------------------------------------\n";

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
      txR,  //Tr: transmission range of a senseor node
      dgen, //p:  num data generators (DG)
      dval, //q:  num date generator items
      ncap, //m:  capacity of non DG nodes
      minR, //a:  min energy E range
      maxR, //b:  max energy E range
      e0;   //E0: initial energy E (rand val range [a, b]

  bool connected;

  Graph g;  //g:  Graph G = (V,E), where V is the set of sensors in network

  vector<SensorNode> sensors; //

  /**
   *
   */
  SensorNet()
  {

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

  /**
   *
   */
  void GenerateNodes()
  {
    int px, py, dgs;

    stringstream ss;

    for (int i = 0; i < N; ++i)
    {
      px = 1 + (rand() % (xlen-1));// GetPosition(xlen);
      py = 1 + (rand() % (ylen-1));//GetPosition(ylen);

      ss << i; //px << "," << py;

      cout << "Sensor[" << i << "]" << "x,y[" << px << ", " << py << "]" << endl;
      SensorNode n(ss.str());
      n.x = px;
      n.y = py;
      n.mcap = ncap;

      if(rand() % 3 == 0 && dgs < dgen)
      {
        n.dgen = true;
        dgs++;
      }

      sensors.push_back(n);
      ss.str("");
      ss.clear();
    }

    vector<SensorNode>::iterator nit = sensors.begin();

    for ( size_t i = 0; nit != sensors.end(); ++nit, ++i)
    {
      for(size_t j = 0;  j != i && j < sensors.size(); j++)
      {
        float dx = max(sensors[i].x, sensors[j].x) - min(sensors[i].x, sensors[j].x);
        float dy = max(sensors[i].y, sensors[j].y) - min(sensors[i].y, sensors[j].y);
        float dist = sqrt((dx*dx) + (dy*dy));

        if(dist < float(txR))
        {
          //g.add_edge(sensors[i].id, sensors[j].id);
          g.add_edge(sensors[i], sensors[j]);
          cout << "connecting sensors " << i << "---" << j <<endl;
          cout << "dist: " << sensors[i].id << ", " << sensors[j].id << endl
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
  list < vector<Edge> > BICONNECTED_COMP(Graph & G)
  {
    vector<Edge> bccs;
    list< vector<Edge> > abccs;
    VertexMapIt vit = G.VE.begin();
    Vertex* u = 0;

    t = 0;
    for(; vit != G.VE.end(); ++vit)
    {
      u = (Vertex*)&vit->first;
      u->color  = eWhite;
      u->pi     = NIL;
      cout << "BCC      : time:" << t << ", "
        << "Initialized u:" << u->id << endl;
    }

    cout << ln;
    t = 0;

    for(vit = G.VE.begin(); vit != G.VE.end(); ++vit)
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

    if(g.VE.size() == N)
    {
      list < vector<Edge> > abccs = BICONNECTED_COMP(g);
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
    }

    return rval;
  };

  /**
   *
   */
  bool IsConnected() { return connected; };

  /**
   *
   */
  map< pair<string, string>, int> BELLMAN_FORD(Vertex* src, map< pair<string, string>, int> & pd)
  {
    for(VertexMapIt i = G.VE.begin(); i != G.VE.end(); ++i)
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

    cout << "sz: " << G.esize() << endl;
    for(size_t j = 0; j < G.vsize(); ++j)
    {
      for(size_t k = 0; k < G.esize(); ++k)
      {
        Edge* e =  G.get_edge(k);
        Vertex* u = e->u, *v = e->v;
        G.relax(*u, *v, e->cap);
        cout << u->id << " " << u->d << " : "
          << v->id << " " << v->d << " " << v  << " " << u << endl;
      }

    }
    for(size_t k = 0; k < G.esize(); ++k)
    {
      Edge* e = G.get_edge(k);

      if(e)
      {
        if(e->u->d + e->cap < e->v->d)
        {
          cerr << "ERROR: Graph contains a negative weight cycle" << endl;
        }
      }
    }

    for(VertexMapIt i = G.VE.begin(); i != G.VE.end(); ++i)
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
   *
   */
  int EnergyConsumption()
  {
    //pj = 10^-12
    //nj = 10 ^ -9
    //TxPower = 
    int rval = 0;

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
