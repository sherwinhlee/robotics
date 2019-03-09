if __name__== "__main__":
    parent = os.path.realpath('..')
    sys.stdout = open(parent + '/logs/' + logfile + '.txt','w') # PARAMETER
    obj = AccControl(robot='create1',accel=0.05,vel=None,coordsys='s',delay=10)
    obj.control()
    sys.stdout.close()
    plot(obj.x,obj.y,X_LABLIM,Y_LABLIM,'Robot Trajectory')