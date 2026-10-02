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
def SumOfForcesInLocalX(node, unknown_bars):
    # Vector direction of local x axis (pointing away from node along local_x_bar)
    local_x_vec = geom.BarNodeToVector(node, local_x_bar)
    
    # 1. Contributions of external/reaction forces in global X and Y
    net_fy = node.GetNetYForce()
    sum_local_x += net_fy * geom.CosineBars(local_x_bar, [0,1])
    
    net_fx = node.GetNetXForce()
    sum_local_x += net_fy * geom.CosineBars(local_x_bar, [1,0])
    # Project external force vector onto local x direction
    
    # 2. Contributions from known internal member loads
    for bar in node.bars:
            if bar.is_computed:
                sum_local_x += bar.axial_load * geom.CosineBars(local_x_bar, bar)
    
    local_x_bar_force = - sum_local_x   
    
    # Store force and mark bar as visited/computed
    local_x_bar.SetAxialLoad(local_x_bar_force)
    local_x_bar.is_computed = True
    
# Compute unknown force in bar due to sum of the 
# forces in the y direction
def SumOfForcesInLocalY(node, unknown_bars):
    local_x_bar = unknown_bars[0]
    other_bar = unknown_bars[1]
    
    local_x_vec = geom.BarNodeToVector(node, local_x_bar)
    
    # 1. External/reaction forces projected onto local Y axis
    net_fy = node.GetNetYForce()
    sum_local_y = net_fy * geom.SineVectors(local_x_vec, [0, 1])
    
    net_fx = node.GetNetXForce()
    sum_local_y = net_fy * geom.SineVectors(local_x_vec, [1, 0])
    
    
    for bar in node.bars:
        if bar.is_computed:
            sum_local_y += bar.axial_load * geom.SineBars(local_x_bar, bar)
    
    # 3. Solve for the second unknown bar's force in local Y
    sin_other = geom.SineBars(local_x_bar, other_bar)
    other_bar_force = -sum_local_y / sin_other
    
    # Store force and mark second bar as computed
    other_bar.SetAxialLoad(other_bar_force)
    other_bar.is_computed = True
    
    
# Perform the method of joints on the structure
def IterateUsingMethodOfJoints(nodes,bars):
    counter = 0 
    max_iterations = len(bars) * 10 # Prevent infinite loops
   
    while any(not bar.is_computed for bar in bars):
        if counter > max_iterations:
            sys.exit("Method of joints reached maximum iterations without solving all bars")


        for node in nodes:
            unknowns = UnknownBars(node)
            if NodeIsViable(node):
                if len(unknowns) == 2:
                    SumOfForcesInLocalY(node, unknowns)
                    unknowns = UnknownBars(node)
                    
                if len(unknowns) == 1:
                    SumOfForcesInLocalX(node, unknowns[0])
        counter += 1
    
    return bars
