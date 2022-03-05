import sympy as sp, numpy as np
x1 = sp.Symbol('x1')
x2 = sp.Symbol('x2')
x3 = sp.Symbol('x3')

Lf = 1# sp.Symbol('Lf')
A = 1# sp.Symbol('A')
l1= 1# sp.Symbol('l1')
g = 1# sp.Symbol('g')
W = 1# sp.Symbol('W')
ss = sp.Symbol('ss')

# f = sp.Matrix([x2, -(2./3)**(1./3)*(Lf*sp.sqrt(2*g))**(2./3) * (sp.cos(x3)/(W*l1))*x2**(4./3), 0])

# f = sp.Matrix([sp.sqrt(x2**2 + 2*g*(ss-x1) ), -(2./3)**(1./3)*(Lf*sp.sqrt(2*g))**(2./3) * (sp.cos(x3)/(W*l1))*x2**(4./3), 0])
# g = sp.Matrix([0, -(2./3)**(1./3) * sp.tan(x3)*x2 + (1./12)**(1./3) * l1* (Lf*sp.sqrt(2*g))**(2./3) * sp.sec(x3) * A**(-2./3) * x2**(1./3)  ,1 ])

#Simplify:
f = sp.Matrix([sp.sqrt(x2**2 + (ss- x1)), sp.cos(x3)*x2**(4./3), 0])
g = sp.Matrix([0, sp.tan(x3)*x2 + sp.sec(x3) *  x2**(1./3)  ,1 ])




print "f: ", f, "\n g: ", g


dg_dx = sp.Matrix(np.hstack([sp.diff(g,x1), sp.diff(g,x2), sp.diff(g,x3)]))
df_dx = sp.Matrix(np.hstack([sp.diff(f,x1), sp.diff(f,x2), sp.diff(f,x3)]))

adfg = dg_dx*f - df_dx*g; adfg = sp.simplify(adfg)

dadfg_dx = sp.Matrix(np.hstack([sp.diff(adfg,x1), sp.diff(adfg,x2), sp.diff(adfg,x3)]))

ad2fg = dadfg_dx*f - df_dx*adfg; sp.simplify(ad2fg)
print "\nadfg: ", adfg
print "\nad2fg: ", ad2fg

M = sp.Matrix(np.hstack([g,adfg,ad2fg])); M = sp.simplify(M)
print "\n Resultant Matrix: ", M
thedet = sp.simplify(sp.det(M))
print "\n Determinant: ", thedet

#Involutive Check 
dad2fg_dx = sp.Matrix(np.hstack([sp.diff(ad2fg,x1), sp.diff(ad2fg,x2), sp.diff(ad2fg,x3)]))
g_adfg = dadfg_dx*g - dg_dx*adfg; g_adfg = sp.simplify(g_adfg)
g_ad2fg = dad2fg_dx*g - dg_dx*ad2fg; g_ad2fg = sp.simplify(g_ad2fg)

adfg_ad2fg = dad2fg_dx*adfg - dadfg_dx*ad2fg; adfg_ad2fg = sp.simplify(adfg_ad2fg)

print "\n Involutive Checks: ", "\n g_adfg",g_adfg, "\n g_ad2fg", g_ad2fg, "\n adfg_ad2fg", adfg_ad2fg



print "\n\n\n The Original Span: ", M


################# Involutive Analysis ######################################
#  The Original Span:  Matrix([
# [0,
# -0.550321208149104*A**(-0.666666666666667)*l1*x2**0.333333333333333*(Lf*sqrt(g))**0.666666666666667*sec(x3) + 0.873580464736299*x2*tan(x3), 
# -0.40380457618492*A**(-0.666666666666667)*x2**0.666666666666667*(Lf*sqrt(g))**1.33333333333333/W + 0.459642607272694*x2**1.33333333333333*(Lf*sqrt(g))**0.666666666666667*sin(x3)/(W*l1)],
# [0.550321208149104*A**(-0.666666666666667)*l1*x2**0.333333333333333*(Lf*sqrt(g))**0.666666666666667*sec(x3) - 0.873580464736299*x2*tan(x3), 
# 0.60570686427738*A**(-0.666666666666667)*x2**0.666666666666667*(Lf*sqrt(g))**1.33333333333333/W - 1.42114232081097*x2**1.33333333333333*(Lf*sqrt(g))**0.666666666666667*sin(x3)/(W*l1),
# 0.444444444444444*A**(-0.666666666666667)*x2**1.0*(Lf*sqrt(g))**2.0*cos(x3)/(W**2*l1)],
# [1,
# 0, 
# 0]])
# v1 = 
# [0,
# 0.550321208149104*A**(-0.666666666666667)*l1*x2**0.333333333333333*(Lf*sqrt(g))**0.666666666666667*sec(x3) - 0.873580464736299*x2*tan(x3), 
# 1]
# v2 = 
# [-0.550321208149104*A**(-0.666666666666667)*l1*x2**0.333333333333333*(Lf*sqrt(g))**0.666666666666667*sec(x3) + 0.873580464736299*x2*tan(x3), ,
# 0.60570686427738*A**(-0.666666666666667)*x2**0.666666666666667*(Lf*sqrt(g))**1.33333333333333/W - 1.42114232081097*x2**1.33333333333333*(Lf*sqrt(g))**0.666666666666667*sin(x3)/(W*l1),,
# 0]
# v3 = 
# [-0.40380457618492*A**(-0.666666666666667)*x2**0.666666666666667*(Lf*sqrt(g))**1.33333333333333/W + 0.459642607272694*x2**1.33333333333333*(Lf*sqrt(g))**0.666666666666667*sin(x3)/(W*l1),
# 0.444444444444444*A**(-0.666666666666667)*x2**1.0*(Lf*sqrt(g))**2.0*cos(x3)/(W**2*l1),
# 0]
########################## 
# g_adfg Matrix([
# [-0.10095114404623*A**(-1.33333333333333)*l1**2*x2**(-0.333333333333333)*(Lf*sqrt(g))**1.33333333333333*sec(x3)**2 + 0.0906786008764104*A**(-0.666666666666667)*l1*x2**0.333333333333333*(Lf*sqrt(g))**0.666666666666667*tan(x3)*sec(x3) + 0.110437636367411*x2**1.0*tan(x3)**2 + 0.873580464736299*x2**1.0],
# [0.111111111111111*A**(-1.33333333333333)*l1*x2**(-2.22044604925031e-16)*(Lf*sqrt(g))**2.0/(W*cos(x3)) - 0.60570686427738*A**(-0.666666666666667)*x2**0.666666666666667*(Lf*sqrt(g))**1.33333333333333*tan(x3)/W - 1.83496971050112*x2**1.33333333333333*(Lf*sqrt(g))**0.666666666666667*cos(x3)/(W*l1) + 0.413827389690155*x2**1.33333333333333*(Lf*sqrt(g))**0.666666666666667/(W*l1*cos(x3))],
# [0]]) 
#  g_ad2fg Matrix([
# [-0.148148148148148*A**(-1.33333333333333)*l1*x2**(-2.22044604925031e-16)*(Lf*sqrt(g))**2.0/(W*cos(x3)) + 0.572438626152329*A**(-0.666666666666667)*x2**0.666666666666667*(Lf*sqrt(g))**1.33333333333333*tan(x3)/W + 0.995022343904539*x2**1.33333333333333*(Lf*sqrt(g))**0.666666666666667*cos(x3)/(W*l1) - 0.535379736631845*x2**1.33333333333333*(Lf*sqrt(g))**0.666666666666667/(W*l1*cos(x3))],
# [1.0*A**(-2.0)*(0.0815290678739413*A**0.666666666666667*l1**2*x2**0.333333333333333*(Lf*sqrt(g))**2.66666666666667 + 0.0815290678739414*A**0.666666666666667*l1**2*x2**0.333333333333333*(Lf*sqrt(g))**2.66666666666667 - 0.69760096470438*A**1.33333333333333*l1*x2**1.0*(Lf*sqrt(g))**2.0*sin(x3) + 0.253156520259936*A**1.33333333333333*l1*x2**1.0*(Lf*sqrt(g))**2.0*sin(x3) + 0.607301304158604*A**2.0*x2**1.66666666666667*(Lf*sqrt(g))**1.33333333333333*sin(x3)**2 - 0.607301304158604*A**2.0*x2**1.66666666666667*(Lf*sqrt(g))**1.33333333333333*sin(x3)**2)/(W**2*l1**2)],
# [0]]) 
#  adfg_ad2fg Matrix([
# [-0.0815290678739414*A**(-1.33333333333333)*x2**0.333333333333333*(Lf*sqrt(g))**2.66666666666667/W**2 + 0.365529440494337*A**(-0.666666666666667)*x2**1.0*(Lf*sqrt(g))**2.0*sin(x3)/(W**2*l1) - 0.870956748857493*x2**1.66666666666667*(Lf*sqrt(g))**1.33333333333333*sin(x3)**2/(W**2*l1**2)],
# [1.0*A**(-1.33333333333333)*x2**(-1.0)*(1.11022302462516e-16*A**0.666666666666667*l1*x2**2.33333333333333*(Lf*sqrt(g))**2.66666666666667*sin(x3) + 0.210539603083106*A**0.666666666666667*l1*x2**2.33333333333333*(Lf*sqrt(g))**2.66666666666667*sin(x3) + 0.987958888348094*A**1.33333333333333*x2**3.0*(Lf*sqrt(g))**2.0*sin(x3)**2 - 0.987958888348094*A**1.33333333333333*x2**3.0*(Lf*sqrt(g))**2.0*sin(x3)**2 + 0.0897343502633155*l1**2*x2**1.66666666666667*(Lf*sqrt(g))**3.33333333333333)*cos(x3)/(W**3*l1**3)],
# [0]])


# Hence it can be seen that they are combos of v2,v3





################# Determinant analysis: ####################################


"""
Unsimplified: 
 Mdet
-0.244587203621824*A**(-1.33333333333333)*x2**1.33333333333333*(Lf*sqrt(g))**2.66666666666667*cos(x3)*sec(x3)/W**2 + 
0.244587203621824*A**(-1.33333333333333)*x2**1.33333333333333*(Lf*sqrt(g))**2.66666666666667/W**2 - 
0.852272454892948*A**(-0.666666666666667)*x2**2.0*(Lf*sqrt(g))**2.0*sin(x3)/(W**2*l1) + 
0.388257984327244*A**(-0.666666666666667)*x2**2.0*(Lf*sqrt(g))**2.0*cos(x3)*tan(x3)/(W**2*l1) + 
0.65321756164312*x2**2.66666666666667*(Lf*sqrt(g))**1.33333333333333*sin(x3)**2/(W**2*l1**2)

"""



"""
simplifed by hand

-0.244587203621824*A**(-1.33333333333333)*x2**1.33333333333333*(Lf*sqrt(g))**2.66666666666667/W**2 + 

0.244587203621824*A**(-1.33333333333333)*x2**1.33333333333333*(Lf*sqrt(g))**2.66666666666667/W**2 - 

0.852272454892948*A**(-0.666666666666667)*x2**2.0*(Lf*sqrt(g))**2.0*sin(x3)/(W**2*l1) + 

0.388257984327244*A**(-0.666666666666667)*x2**2.0*(Lf*sqrt(g))**2.0*sin(x3)/(W**2*l1) + 

0.65321756164312*x2**2.66666666666667*(Lf*sqrt(g))**1.33333333333333*sin(x3)**2/(W**2*l1**2)

"""


#can only be zero if x2 = 0, if x3 = 0 and x2 = 0, note that the psuedo inverse of g(x) also implicitly requires x3 != -pi/2,pi/2
#hence, pouring must start (x2 !=0, x3!=0), and x3 in [-pi/2,pi/2] 




"""
[non consequential term:]
2.77555756156289e-17*A**(-1.33333333333333)*x2**1.33333333333333*(Lf*sqrt(g))**2.66666666666667/W**2 - 

[driving term]
0.464014470565704*A**(-0.666666666666667)*x2**2.0*(Lf*sqrt(g))**2.0*sin(x3)/(W**2*l1) + 

[driving term]
0.65321756164312*x2**2.66666666666667*(Lf*sqrt(g))**1.33333333333333*sin(x3)**2/(W**2*l1**2)
"""




"""
solns:  sp.solve(thedet,x2,x3)
[{x2: 0.0},
 {x2: -4.79184993325651e-34*(5.8001808820713e+21*A**(-0.666666666666667)*Lf**2*g*l1*(Lf*g**0.5)**(-1.33333333333333)/sin(x3) - 5.8001808820713e+21*Lf**2*g*l1*(Lf*g**0.5)**(-1.33333333333333)*(A**2.66666666666667)**0.5/(A**2*sin(x3)))**(3/2)},
 {x2: 4.79184993325651e-34*(5.8001808820713e+21*A**(-0.666666666666667)*Lf**2*g*l1*(Lf*g**0.5)**(-1.33333333333333)/sin(x3) - 5.8001808820713e+21*Lf**2*g*l1*(Lf*g**0.5)**(-1.33333333333333)*(A**2.66666666666667)**0.5/(A**2*sin(x3)))**(3/2)},
 {x2: -4.79184993325651e-34*(5.8001808820713e+21*A**(-0.666666666666667)*Lf**2*g*l1*(Lf*g**0.5)**(-1.33333333333333)/sin(x3) + 5.8001808820713e+21*Lf**2*g*l1*(Lf*g**0.5)**(-1.33333333333333)*(A**2.66666666666667)**0.5/(A**2*sin(x3)))**(3/2)},
 {x2: 4.79184993325651e-34*(5.8001808820713e+21*A**(-0.666666666666667)*Lf**2*g*l1*(Lf*g**0.5)**(-1.33333333333333)/sin(x3) + 5.8001808820713e+21*Lf**2*g*l1*(Lf*g**0.5)**(-1.33333333333333)*(A**2.66666666666667)**0.5/(A**2*sin(x3)))**(3/2)}]

and sp.solve(thedet)

"""


"""Solving explicitly for x3: 
sp.solve(thedet,x3)

Soln1:
-asin(l1*(-0.355176052981881*Lf**2*g*sqrt(A**2.66666666666667)/(A**2*x2**(2/3)*(Lf*g**0.5)**(4/3)) + 0.355176052981881*Lf**2*g/(A**(2/3)*x2**(2/3)*(Lf*g**0.5)**(4/3)))) + 3.14159265358979,

Soln2:
 -asin(l1*(0.355176052981881*Lf**2*g*sqrt(A**2.66666666666667)/(A**2*x2**(2/3)*(Lf*g**0.5)**(4/3)) + 0.355176052981881*Lf**2*g/(A**(2/3)*x2**(2/3)*(Lf*g**0.5)**(4/3)))) + 3.14159265358979,

Soln3:
 asin(l1*(-0.355176052981881*Lf**2*g*sqrt(A**2.66666666666667)/(A**2*x2**(2/3)*(Lf*g**0.5)**(4/3)) + 0.355176052981881*Lf**2*g/(A**(2/3)*x2**(2/3)*(Lf*g**0.5)**(4/3)))),

Soln4:
 asin(l1*(0.355176052981881*Lf**2*g*sqrt(A**2.66666666666667)/(A**2*x2**(2/3)*(Lf*g**0.5)**(4/3)) + 0.355176052981881*Lf**2*g/(A**(2/3)*x2**(2/3)*(Lf*g**0.5)**(4/3))))]

 """