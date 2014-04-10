#! /usr/bin/env python3.3
####################################### Plot 1
from decimal import *
import matplotlib.pyplot as plt
import numpy as np
import os
from pylab import * 
from matplotlib.font_manager import FontProperties

fontP = FontProperties()
fontP.set_size('small')

if not os.path.exists('../plots'):
	os.makedirs('../plots')

f=open('../data/g06_lab05data_02.csv','r')   #CSV1
error_bars=[]
NoOfIter = 40
NoOfReruns = 20
iteration_number=[(x+1) for x in range(NoOfIter)]
step_time_avg=[0 for x in range(NoOfIter)]
collision_time_avg=[0 for x in range(NoOfIter)]
velocity_time_avg=[0 for x in range(NoOfIter)]
position_time_avg=[0 for x in range(NoOfIter)]
loop_time_avg=[0 for x in range(NoOfIter)]
sum_of_avg=[0 for x in range(NoOfIter)]
step_time_rollno=[]
maxIndLT=-1
minIndLT=-1
maxLT=0
minLT=10000
for i in range(NoOfIter):  #No of iterations
	maxi=0
	mini=10000
	step_time_sum=0
	collision_time_sum=0
	velocity_time_sum=0
	position_time_sum=0
	loop_time_sum=0	
	for j in range(NoOfReruns):  #No of reruns
		line=f.readline()
		words=line.split(',')
		step_value=float(words[2])
		if i==19:  #Shuld take 49
			step_time_rollno.append(step_value)
		if step_value>maxi:
			maxi=step_value
		if step_value<mini:
			mini=step_value
		step_time_sum+=float(words[2])
		collision_time_sum+=float(words[3])
		velocity_time_sum+=float(words[4])
		position_time_sum+=float(words[5])
		loop_time_sum+=float(words[6])
	error_bars.append(maxi-mini)
	step_time_avg[i]=step_time_sum/NoOfReruns
	collision_time_avg[i]=collision_time_sum/NoOfReruns
	velocity_time_avg[i]=velocity_time_sum/NoOfReruns
	position_time_avg[i]=position_time_sum/NoOfReruns
	loop_time_avg[i]=loop_time_sum/NoOfReruns
	if loop_time_avg[i]>maxLT:
		maxLT=loop_time_avg[i]
		maxIndLT=i
	if loop_time_avg[i]<minLT:
		minLT=loop_time_avg[i]
		minIndLT=i
plt.bar(iteration_number,step_time_avg,0.7,align='center',label='Step time avg')  
plt.plot(iteration_number,loop_time_avg,'r',label='Loop time avg')
plt.xlabel('Iteration Number')
plt.ylabel('Step time and loop time averages')
plt.title('Plot 1')
#plt.legend()
plt.annotate('{}'.format("max"), xy=(maxIndLT,maxLT), xytext=(-10, 10), ha='right', textcoords='offset points',arrowprops=dict(arrowstyle='->', shrinkA=0))
plt.annotate('{}'.format("min"), xy=(minIndLT,minLT), xytext=(-10, 10), ha='right', textcoords='offset points',arrowprops=dict(arrowstyle='->', shrinkA=0))
plt.legend(prop = fontP,loc='upper center')
plt.savefig('../plots/g06_lab09_plot01.png')
plt.clf() #Clearing to use it later

############################################### Plot 2

for i in range(NoOfIter):
	sum_of_avg[i]=collision_time_avg[i]+velocity_time_avg[i]+position_time_avg[i]
plt.plot(iteration_number,step_time_avg,'r',label='Step time avg')
plt.plot(iteration_number,collision_time_avg,'g',label='Collision time avg')
plt.plot(iteration_number,velocity_time_avg,'b',label='Velocity time avg')
plt.plot(iteration_number,position_time_avg,'k',label='Position time avg')
plt.plot(iteration_number,sum_of_avg,'m',label='Sum of averages')
plt.legend(prop = fontP)
plt.title('Plot 2')
plt.xlabel('Iteration Number')
plt.ylabel('Various averages')
plt.savefig('../plots/g06_lab09_plot02.png')
plt.clf() #Clearing to use it later

##############################################  Plot 3

plt.errorbar(iteration_number,step_time_avg,yerr=error_bars,fmt='o',label='errorbars')
plt.plot(iteration_number,step_time_avg,'r',label='Step time avg')
plt.xlabel('Iteration number')
plt.ylabel('Step time with error bars')
plt.title('Plot 3')
plt.legend(prop = fontP)
plt.savefig('../plots/g06_lab09_plot03.png')
plt.clf() #Clearing to use it later

############################################# Plot 5

f=open('../data/g06_lab05data_random.csv','r') #RandomCSV
random_step_average=[0 for x in range(NoOfIter)] #No of iterations in random csv
NoOfRerunsR = 10
for i in range(NoOfIter):
	random_step_sum=0
	for j in range(NoOfRerunsR): #No. of values in each iteration
		line=f.readline()
		words=line.split(',')
		random_step_sum+=float(words[2])
	random_step_average[i]=random_step_sum/NoOfRerunsR

fit1 = polyfit(iteration_number,random_step_average,1)
fit_fn1 = poly1d(fit1) # fit_fn is now a function which takes in x and returns an estimate for y
plot(iteration_number,fit_fn1(iteration_number),'b',linestyle = "-.",label='Best Fit Random')

fit2 = polyfit(iteration_number,step_time_avg,1)
fit_fn2 = poly1d(fit2) # fit_fn is now a function which takes in x and returns an estimate for y
plot(iteration_number,fit_fn2(iteration_number),'g',linestyle = "--",label='Best Fit Full')

plt.plot(iteration_number,random_step_average,color = 'r',linestyle = "None",label='Step time avg in random file', marker='o')
plt.plot(iteration_number,step_time_avg,color = 'b',linestyle = "None",label='Step time avg', marker='o')
plt.title('Plot 5')
plt.legend(prop = fontP)
plt.xlabel('Iteration number')
plt.ylabel('Step time averages of both csv files')
plt.savefig('../plots/g06_lab09_plot05.png')
plt.clf() #Clearing to use it later

############################################# Plot 4
plt.hist(step_time_rollno,bins=NoOfReruns/5,cumulative='True',histtype='step',label="Cumulative graph")
plt.hist(step_time_rollno,bins=NoOfReruns/5,label="Frequency plot",align='mid')
plt.legend(prop = fontP,loc='upper center')
plt.xlabel('Bins')
plt.ylabel('Step time average frequency')
plt.title('Plot 4')
plt.savefig('../plots/g06_lab09_plot04.png')
plt.clf()

