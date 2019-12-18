
import sys 
sys.path.append('../..')

def aes(bus,input_data):

    import os
    import cosim
    from aesmod.agents import BuildingAgent
    from aesmod.agents import StorageAgent
    from aesmod.agents import WindAgent
    from aesmod.agents import OpenDSSAgent
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # initialize environment to run co-simulation
    env = cosim.Environment(input_data['startTime'],
                            input_data['stopTime'],
                            bus=bus,
                            working_directory=os.path.join(dir_path, "working/env"))

    # add in opendss agent
    _ = OpenDSSAgent(env,'opendss',input_data['feeder_file'],input_data=input_data)
    _ = StorageAgent(env, "storage_{0:02d}".format(00),input_data,6)

    # Constructing agents
    for neighborhood in range(3):
        # print('++++++++++++++++++++++++++++++++++++++++++++++')
        # print('Assigning external loads/generators...')
        # print('++++++++++++++++++++++++++++++++++++++++++++++')

        _ = BuildingAgent(env, "building_{0:02d}".format(neighborhood),input_data,neighborhood+4)
        #_ = StorageAgent(env, "storage_{0:02d}".format(neighborhood),input_data,neighborhood+4)

        # put wind on the generator nodes
        _ = WindAgent(env, "wind_{0:02d}".format(neighborhood),input_data,neighborhood+0)

    print('Agent mapping...')
    env.agent_mapping()
    print('Agent execute...')
    env.execute()
    print('stop...')
    bus.stop()

