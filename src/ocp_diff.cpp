/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

 /**
 *    \file   examples/getting_started/simple_ocp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */

#include "ros/ros.h"
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main(int argc, char** argv){
    ros::init(argc, argv, "ocp_diff");
    ros::NodeHandle nh;
    USING_NAMESPACE_ACADO

    DifferentialState        s,v      ;     // the differential states
    Control                  u          ;     // the control input u
    DifferentialEquation     f( 0.0, 2.0 );     // the differential equation

//  -------------------------------------
    OCP ocp( 0.0, 2.0, 30);                        // time horizon of the OCP: [0,T]
    Function fu;
    fu << u;
    ocp.minimizeMayerTerm( u*u );               // the time T should be optimized

    f << dot(s) == v;                         // an implementation
    f << dot(v) == u;             // of the model equations
    // f << dot(m) == -0.01*u*u;                 // for the rocket.

    ocp.subjectTo( f                   );     // minimize T s.t. the model,
    ocp.subjectTo( AT_START, s ==  0.0 );     // the initial values for s,
    ocp.subjectTo( AT_START, v ==  0.0 );     // v,
    // ocp.subjectTo( AT_START, m ==  1.0 );     // and m,

    ocp.subjectTo( AT_END  , s == 10.0 );     // the terminal constraints for s
    ocp.subjectTo( AT_END  , v ==  0.0 );     // and v,

    // ocp.subjectTo( -0.1 <= v <=  1.7   );     // as well as the bounds on v
    // ocp.subjectTo( -1.1 <= u <=  1.1   );     // the control input u,
    // ocp.subjectTo(  5.0 <= T <= 15.0   );     // and the time horizon T.
//  -------------------------------------

    // GnuplotWindow window;
    //     window.addSubplot( s, "THE DISTANCE s"      );
    //     window.addSubplot( v, "THE VELOCITY v"      );
    //     window.addSubplot( m, "THE MASS m"          );
    //     window.addSubplot( u, "THE CONTROL INPUT u" );

    OptimizationAlgorithm algorithm(ocp);     // the optimization algorithm
    // algorithm << window;
    // algorithm.set(INTEGRATOR)
    ros::Time ts = ros::Time::now();
    algorithm.solve();                        // solves the problem.
    ROS_INFO("Time taken: %f seconds", (ros::Time::now()-ts).toSec());

    VariablesGrid states;
    algorithm.getDifferentialStates(states);
    ROS_INFO("No of states = %i", states.getNumPoints());
    return 0;
}
