from math import sqrt
def sdf(e):
    grp=e[0]
    matrix=e[1]
    # leader=rospy.get_param('/'+grp+'_leader')
    leader=0
    # x=matrix['r'+str(leader)][]
    # bs_lat = rospy.get_param("/bs/lat")
    # bs_lon = rospy.get_param("/bs/lon")
    bs_lat=0
    bs_lon=0
    dis =sqrt((matrix['r'+str(leader)][0]-bs_lat)**2+(matrix['r'+str(leader)][1]-bs_lon)**2)
    return dis

if __name__ == "__main__":
    q=[['grp1',{'r0':[5,5],'r1':[4,4],'r2':[3,3]}],['grp1',{'r0':[-5,-5],'r1':[-3,-3],'r2':[-4,-4]}],['grp1',{'r0':[-2,1],'r1':[1,0],'r2':[4,1]}],
    ['grp1',{'r0':[0,2],'r1':[2,2],'r2':[-1,-1]}]]
    q.sort(key=sdf)
    for i in q:
        print(str(i)+" ")