#include <iostream>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>

void creates(void);/*std::string, double, double, double, 
                    double, double, double,
                    double, double, double, double);*/
void updates(std::string, double, double, double, 
                   double, double, double,
                   double, double, double, double);
void updates_check(int, double, double, double, 
                   double, double, double,
                   double, double, double, double);
void updates_local(int, double, double, double); 
void updates_global(int, double, double, double);

void creates_sensor(void);/*std::string, double, double, double, 
                                 double, double, double,
                                 double, double, double, 
                                 double, double);*/
void updates_sensor(std::string, double, double, double, 
                                 double, double, double,
                                 double, double, double, 
                                 double, double);
void updates_check_ss(int, double, double, double, 
                           double, double, double,
                           double, double, double, 
                           double, double);                                 