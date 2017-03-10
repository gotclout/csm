#include <iostream>
#include <string>
#include <math.h>
#include <cstdlib>
//#include <stdlib.h>
#include <ctime>


#include "SensorNet.h"

using namespace std;

/**
 *
 */
int get_area(int & px, int & py)
{
  cout << "Enter the network area x value:" << endl;
  cin >> px;
  cout << "Enter the network area y value:" << endl;
  cin >> py;

  return px * py;
}
/**
 *
 */
int& get_nodes(int & pn)
{
  cout << "Enter the number of sensor nodes: " << endl;
  cin >> pn;

  return pn;
}

bool set_input(SensorNet* & pNet)
{
    pNet->xlen = 50;
    pNet->ylen = 50;
    pNet->area = 50*50;
    pNet->N    = 30;
    pNet->txR  = 16;
    pNet->dgen = 5;
    pNet->ncap = 2;
    pNet->dval = 5;
    pNet->minR = 2;
    pNet->maxR = 9;

    pNet->InitE();
    cout << "Sensor network constrained to area: "
         << pNet->area << "m" << endl;
    cout << "Sensor network consists of " << pNet->N << " nodes" << endl
         << "Sensor node transmission range: " << pNet->txR << endl
         << "Sensor net consists of: " << pNet->dgen << " DG nodes" << endl
         << "and " << pNet->N - pNet->dgen << " non-dg sensors with " << endl
         << "capacity " << pNet->ncap << endl
         << "Each DG sensor has " << pNet->dval << " data items" << endl
         << "Sensors in the network have a max energy range " << pNet->maxR
         << endl << "and min energy range " << pNet->minR << " with initial "
         << "energy values e0: " << pNet->e0 << endl;
    cout << "Generating nodes: " << endl;

    pNet->GenerateNodes();

    vector<SensorNode> sensors = pNet->GetSensors();
    vector<SensorNode>::iterator nit = sensors.begin();

    cout << "Sensors..." << endl;
    for( ; nit != sensors.end(); ++nit)
    {
      cout << *nit << endl;
    }

    cout << "G..." << endl;

    cout << pNet->g << endl;

    if(pNet->IsConnected())
    {
      cout << "Network is connected" << endl;
      string u, v;
      //cout << "Enter two nodes to compute energy consumption (3 4):" << endl;
      //cin >> u >> v;
      int e = pNet->EnergyConsumption("8", "16");
      cout << "Consumption " << u << ":" << v << " " << e << "J" << endl;
    }
    return true;
}


/**
 *
 */
bool get_input(SensorNet* & pNet)
{
  bool rval = true;

  if(pNet)
  {
    pNet->area = get_area(pNet->xlen, pNet->ylen);
    pNet->N    = get_nodes(pNet->N);
    cout << "Enter the sensor transmission range:"  << endl;
    cin >> pNet->txR;
    cout << "Enter the number of DG sensors:" << endl;
    cin >> pNet->dgen;
    cout << "Enter the capacity of non-dg sensors:" << endl;
    cin >> pNet->ncap;
    cout << "Enter the number of data items for each DG sensor:" << endl;
    cin >> pNet->dval;
    cout << "Enter the min and max sensor energy range (min max):" << endl;
    cin >> pNet->minR >> pNet->maxR;

    int e0 = pNet->InitE();

    cout << "Sensor network constrained to area: "
         << pNet->area << "m" << endl;
    cout << "Sensor network consists of " << pNet->N << " nodes" << endl
         << "Sensor node transmission range: " << pNet->txR << endl
         << "Sensor net consists of: " << pNet->dgen << " DG nodes" << endl
         << "and " << pNet->N - pNet->dgen << " non-dg sensors with " << endl
         << "capacity " << pNet->ncap << endl
         << "Each DG sensor has " << pNet->dval << " data items" << endl
         << "Sensors in the network have a max energy range " << pNet->maxR
         << endl << "and min energy range " << pNet->minR << " with initial "
         << "energy values e0: " << pNet->e0 << endl;
    cout << "Generating nodes: " << endl;

    pNet->GenerateNodes();

    vector<SensorNode> sensors = pNet->GetSensors();
    vector<SensorNode>::iterator nit = sensors.begin();

    cout << "Sensors..." << endl;
    for( ; nit != sensors.end(); ++nit)
    {
      cout << *nit << endl;
    }

    cout << "..." << endl;
    if(pNet->IsConnected())
    {
      cout << "Network is connected" << endl;
      string u, v;
      cout << "Enter two nodes to compute energy consumption u v" << endl;
      cin >> u >> v;
      int e = pNet->EnergyConsumption(u, v);
      cout << "Consumption " << u << ":" << v << " " << e << "J" << endl;
    }
  }
  else
  {
    rval = false;
  }
  return rval;
}

bool GenRandomNodes(SensorNet* & pNet)
{
  bool rval = true;

  if(!pNet)
  {
    rval = false;

  }
  else
  {
    ;
  }

  return rval;
}
/**
 *
 */
SensorNet* GenSensorNet()
{
  SensorNet* net = new SensorNet;
  //if(get_input(net) == false)
  if(get_input(net) == false)
  {
    cerr << "Error: failed to generate sensor net" << endl;
  }
  else cout << "Sensor network successfully generated" << endl;
  return net;
}

/**
 *
 */
int main(int argc, char** argv)
{
  cout << "start";
  srand(time(0));


  //int x, y, xy, N, tr, p, q, m, minR, maxR, e0; //pq <= (N-p)m

  if(argc > 1)
  {
    cerr << "Usage: " << endl;
  }
  else
  {/*
    xy = get_area(x, y);
    N  = get_nodes(N);
    cout << "Sensor network constrained to area: "
         << xy << "m" << endl;
    cout << "Sensor network consists of " << N << " nodes" << endl;*/
    cout << "Generating sensor network, please provided the following data..."
         << endl;
    SensorNet* n = GenSensorNet();
    if(n) cout << "Success!" << endl;
  }

  return 0;
}
