# Area_Coverage

This project is aimed at using optimal control to solve the problem of coverage path planning. 

The method is based on the problem as described in:
"""
Manerikar, Ankit, Debasmit Das, and Pranay Banerjee. "Optimal Control for Constrained Coverage Path Planning." 
arXiv preprint arXiv:1708.03055 (2017).
"""

The code from that resource is also in this repository, in the subfolder Coverage_Path_Planning- 
  However, the authors utilize a MATLAB optimization framework called TOMLAB
  This code aims to circumvent TOMLAB by formulating the problem as a chain of TPBVPs and solving 
  them with bvp4c() in MATLAB
