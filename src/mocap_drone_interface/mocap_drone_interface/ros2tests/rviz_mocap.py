from mocap_drone_interface.library.nodeflight import MocapCaller

def main():
    ids= [10, 11]
    MocapCaller("10.131.77.150", "10.131.196.172", ids)

if __name__ == '__main__':
    main()