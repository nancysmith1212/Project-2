#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 12:37:32 2021

@author: kendrick shepherd
"""

import sys

import Geometry_Operations as geom

# Determine the unknown bars next to this node
def UnknownBars(node):
    unknown = []
    for bar in node.bars:
        if not bar.is_computed:
            unknown.append(bar)
    return unknown 

# Determine if a node if "viable" or not
def NodeIsViable(node):
    unknowns = UnknownBars(node)
    if len(unknowns) == 1 or len(unknowns) == 2:
        return True
    return False
    
# Compute unknown force in bar due to sum of the
# forces in the x direction
def SumOfForcesInLocalX(node, local_x_bar):
    # Vector direction of local x axis (pointing away from node along local_x_bar)
    local_x_vec = geom.BarNodeToVector(node, local_x_bar)
    
    # 1. Contributions of external/reaction forces in global X and Y
    net_fx = node.GetNetXForce()
    net_fy = node.GetNetYForce()
    net_ext_vec = [net_fx, net_fy]
    
    # Project external force vector onto local x direction
    
    # 2. Contributions from known internal member loads
    for bar in node.bars:
        if bar != local_x_bar and bar.is_computed:
            cos_theta = geom.CosineBars(local_x_bar, bar)
            sum_local_x += bar.axial_load * cos_theta
            
    # 3. Equilibrium: sum_local_x + unknown_force = 0 => unknown_force = -sum_local_x
    unknown_force = -sum_local_x
    
    # Store force and mark bar as visited/computed
    local_x_bar.SetAxialLoad(unknown_force)
    local_x_bar.is_computed = True
    
# Compute unknown force in bar due to sum of the 
# forces in the y direction
def SumOfForcesInLocalY(node, unknown_bars):
    local_x_bar = unknown_bars[0]
    other_bar = unknown_bars[1]
    
    local_x_vec = geom.BarNodeToVector(node, local_x_bar)
    
    # 1. External/reaction forces projected onto local Y axis
    net_fx = node.GetNetXForce()
    net_fy = node.GetNetYForce()
    
    # Perpendicular unit vector to local x (-sin, cos) in 2D
    rx = local_x_vec[0]
    ry = local_x_vec[1]
    mag = geom.VectorTwoNorm(local_x_vec)
    local_y_unit = [-ry / mag, rx / mag]
    
    sum_local_y = net_fx * local_y_unit[0] + net_fy * local_y_unit[1]
    
    # 2. Contributions from known bars
    for bar in node.bars:
        if bar not in unknown_bars and bar.is_computed:
            sin_theta = geom.SineBars(local_x_bar, bar)
            sum_local_y += bar.axial_load * sin_theta
    
    # 3. Solve for the second unknown bar's force in local Y
    sin_other = geom.SineBars(local_x_bar, other_bar)
    other_bar_force = -sum_local_y / sin_other
    
    # Store force and mark second bar as computed
    other_bar.SetAxialLoad(other_bar_force)
    other_bar.is_computed = True
    
    # 4. Now solve the first unknown bar using local X equilibrium 
    SumOfForcesInLocalX(node, local_x_bar)
    
# Perform the method of joints on the structure
def IterateUsingMethodOfJoints(nodes,bars):
    counter = 0 
    max_interations = len(bars) * 10 # Prevent infinite loops
    
    while True:
        # Stop condition: check if all bars are computed
        all_bars_computed = True
        for bar in bars:
            all_bars_computed = False
            break
        
        if all_bars_computed:
            break
        
        # Safety check for infinite loop
        counter += 1
        if counter > max_iterations:
            sys.exit("Method of joints reached maximum iterations without solving all bars")
            
        progress_made = False 
        # Iterate through nodes
        for node in nodes:
            if NodeIsViable(node):
                unknowns = UnknownBars(node)
                if len(unknowns) == 1:
                    SumOfForcesInLocalX(node, unknowns[0])
                    progress_made = True
                elif len(unknowns) == 2:
                    SumOfForcesInLocalY(node, unknowns)
                    progress_made = True
        if not progress_made: 
            sys.exit("Stuck in solver loop: structure may be unstable or indeterminate")
