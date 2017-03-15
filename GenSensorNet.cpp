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
    pNet->xlen = 20;
    pNet->ylen = 10;
    pNet->area = 20*10;
    pNet->N    = 15;
    pNet->txR  = 15;
    pNet->dgen = 5;
    pNet->ncap = 4;
    pNet->dval = 2;
    pNet->minR = 15;
    pNet->maxR = 20;

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
      string u = "5", v = "10";
      cout << "Enter two nodes to compute energy consumption u v:" << endl
           << u << " " << v << endl;
      double e = pNet->EnergyConsumption(u, v);
      cout << "Consumption " << u << ":" << v << " " << e << "J" << endl;
      pNet->Plot();
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
      cout << "Enter two nodes to compute energy consumption u v:" << endl;
      cin >> u >> v;
      double e = pNet->EnergyConsumption(u, v);
      cout << "Consumption " << u << ":" << v << " " << e << "J" << endl;
      pNet->Plot();
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
SensorNet* GenSensorNet(bool test = false)
{
  SensorNet* net = new SensorNet;
  if(test)
  {
    if(set_input(net) == false)
    {
      cerr << "Error: failed to generate sensor net" << endl;
    }
    else cout << "Sensor network successfully generated" << endl;
  }
  else
  {
    if(get_input(net) == false)
    {
      cerr << "Error: failed to generate sensor net" << endl;
    }
    else cout << "Sensor network successfully generated" << endl;
  }
  return net;
}

/**
 *
 */
int main(int argc, char** argv)
{
  cout << "start" << endl;
  srand(time(0));

  bool test = false;
  if(argc > 1)
  {
    test = true;
  }
  cout << "Generating sensor network, please provided the following data..."
       << endl;
  SensorNet* n = GenSensorNet(test);
  if(n) cout << "Success!" << endl;

  cin.get();
  return 0;
}
