import os, csv, pickle
import numpy as np
from numpy import genfromtxt
#from skimage.transform import resize


def bag_2_csv(dataset_path, robot):
	"""
	Convert ros bag files to csv file for topic `joint_states`
	Save CSV file in same location as bag file
	"""
	for root, dirs, filenames in os.walk(path):
	    for a_file in filenames:
	        if a_file.endswith(".bag"):
	        	source_file = root+os.sep+a_file
	        	#print(source_file) # source file

	        	source_file_list = source_file.split(os.sep)
	        	
	        	target_file = path+os.sep+os.sep.join(source_file_list[-4:])[:-3]
	        	print(target_file)

	        	if robot == "Fetch":
	        		command = "rostopic echo -b "+source_file+" -p /joint_states > "+target_file+"csv"
		        	os.system(command)
		        else:
		        	# rostopic echo -b baxter_pick_and_place__model4__2018-09-26-14-09-10.bag -p /robot/joint_states > data.csv
		        	command = "rostopic echo -b "+source_file+" -p /robot/joint_states > "+target_file+"csv"
		        	os.system(command)


def bag_2_csv_new(dataset_path):
	"""
	Convert ros bag files to csv file for topic `joint_states`
	Save CSV file in same location as bag file
	"""
	for root, dirs, filenames in os.walk(dataset_path):
		for a_file in filenames:
			if a_file.endswith(".bag"):
				os.makedirs(root+os.sep+"CSVs", exist_ok=True)

				source_file = root+os.sep+a_file
				source_file_list = source_file.split(os.sep)
				target_file = root+os.sep+"CSVs"+os.sep+source_file_list[-1][:-3]
				print(target_file)

				# rostopic echo -b baxter_pick_and_place__model4__2018-09-26-14-09-10.bag -p /robot/joint_states > data.csv
				command = "rostopic echo -b "+source_file+" -p /robot/joint_states > "+target_file+"csv"
				#print(command)
				os.system(command)


def get_baxter_features(csv_file):
	"""
	For baxter, return position, velocity, effort for left arm's left gripper, right gripper and 7 joints
	"""
	my_data = genfromtxt(csv_file, delimiter=',')

	position = my_data[1:, 24:32+1]
	velocity = my_data[1:, 43:51+1]
	effort = my_data[1:, 62:70+1]

	return np.concatenate((position, velocity, effort), axis=1)


def get_sawyer_features(csv_file):
	"""
	For sawyer, return position, velocity, effort for left gripper, right gripper and 7 joints
	"""
	my_data = genfromtxt(csv_file, delimiter=',')

	position = my_data[1:, 15:23+1]
	velocity = my_data[1:, 25:33+1]
	effort = my_data[1:, 35:43+1]

	return np.concatenate((position, velocity, effort), axis=1)


def read_csv_of_each_interaction(dataset_path):
	"""
    Read each csv file and save its features.
    Its save in a dictionary of dictionary of list:
    {"interaction1": {"block1": [data1, data1, ...]}, "interaction2": {"block2": [data1, data1, ...]}}
    """
	baxter = {}
	sawyer = {}
	for root, dirs, filenames in os.walk(path):
	    for a_file in filenames:
	        if a_file.endswith(".csv"):
	        	source_file = root+os.sep+a_file
	        	#print(source_file) # source file

	        	source_file_list = source_file.split(os.sep)
	        	#print(source_file_list)
	        	filename = source_file_list[-1]
	        	interaction = filename.split("_")[1]
	        	#print(interaction)
	        	block = int(filename.split("_")[3])
	        	#print(block)
	        	robot = source_file_list[-2]
	        	#print(robot)
                
	        	if robot == "Baxter":
	        		bax_features = get_baxter_features(source_file)
	        		baxter.setdefault(interaction, {}).setdefault(block, []).append(bax_features)
	        	elif robot == "Sawyer":
	        		saw_features = get_sawyer_features(source_file)
	        		sawyer.setdefault(interaction, {}).setdefault(block, []).append(saw_features)

	return baxter, sawyer


def save_datasets(examples, labels, db_file_name, path):

	output_file = open(path+os.sep+db_file_name, "wb")
	pickle.dump(examples, output_file)
	pickle.dump(labels, output_file)
	output_file.close()


def discretize_datasets(examples, temporal_bins = 10, features = 27):
	"""
	Discretized examples into given temporal bins
	"""

	discretized_examples = []
	for a_example in examples:
		frames = a_example.shape[0]

		# Fix if number of frames is less than temporal_bins
		if frames < temporal_bins:
			print(frames, " is less than "+str(temporal_bins)+" frames")
			a_example = resize(a_example, (temporal_bins, features))
			frames = a_example.shape[0]

		size = frames//temporal_bins
		#print(frames)

		discretized_example = []
		for a_bin in range(temporal_bins):
		    
		    dis_features = []
		    for a_feature in range(features):   		        
		        mean = np.mean(a_example[size*a_bin:size*(a_bin+1)][:, a_feature], axis=0)
		        #print("mean: ", mean)
		        if str(mean) == "nan":
		        	print("mean: ", mean)
		        	print(a_example[size*a_bin:size*(a_bin+1)][:, a_feature])
		        	break
		        dis_features.append(mean)
		    discretized_example.append(dis_features)

		discretized_examples.append(discretized_example)

	return np.array(discretized_examples)


def save_datasets_for_a_robot(robot_name, robot_data, path, temporal_bins = 10, features = 27):
	"""
	Save binary data set files with examples and object labels
	"""
	for interaction in sorted(robot_data):
	    examples = []
	    labels = []
	    print(interaction)
	    all_lenghts = []
	    for a_block in sorted(robot_data[interaction]):
	        print(a_block)
	        robot_data[interaction][a_block] = np.array(robot_data[interaction][a_block])	        
	        for i_example in range(len(robot_data[interaction][a_block])):
	            examples.append(robot_data[interaction][a_block][i_example])
	            labels.append(a_block)
	            #print("len(robot_data[interaction][a_block][i_example]): ", len(robot_data[interaction][a_block][i_example]))
	            all_lenghts.append(len(robot_data[interaction][a_block][i_example]))
	    
	    examples = np.array(examples)
	    labels = np.array(labels)
	    mean_lenght = int(np.mean(np.array(all_lenghts)))
	    print("mean_lenght: ", mean_lenght)

	    db_file_name = robot_name+"_"+interaction+"_"+"variablesize.bin"
	    save_datasets(examples, labels, db_file_name, path)

	    discretized_examples = discretize_datasets(examples, temporal_bins, features)
	    db_file_name = robot_name+"_"+interaction+"_"+"discretized.bin"
	    save_datasets(discretized_examples, labels, db_file_name, path)

	    examples = []
	    for a_block in sorted(robot_data[interaction]):
	    	for i_example in range(len(robot_data[interaction][a_block])):
	    		examples.append(resize(robot_data[interaction][a_block][i_example], (mean_lenght, 27)))
	    db_file_name = robot_name+"_"+interaction+"_"+"fixedsize.bin"
	    examples = np.array(examples)
	    save_datasets(examples, labels, db_file_name, path)


if __name__ == "__main__":
	#path = r".."+os.sep+"Sim_KnoTraBots_datasets_Shake_and_Hold2"
	path = r".."+os.sep+"rosbagfiles_Y50"
	bag_2_csv(path, "Fetch")

	#baxter, sawyer = read_csv_of_each_interaction(path)

	#temporal_bins = 10
	#features = 27
	#save_datasets_for_a_robot("baxter", baxter, path, temporal_bins, features)
	#save_datasets_for_a_robot("sawyer", sawyer, path, temporal_bins, features)
