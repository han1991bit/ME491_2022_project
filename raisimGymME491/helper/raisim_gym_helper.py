from shutil import copyfile
import datetime
import os
import ntpath
import torch


class ConfigurationSaver:
    def __init__(self, log_dir, save_items):
        self._data_dir = log_dir + '/' + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        os.makedirs(self._data_dir)

        if save_items is not None:
            for save_item in save_items:
                base_file_name = ntpath.basename(save_item)
                copyfile(save_item, self._data_dir + '/' + base_file_name)

    @property
    def data_dir(self):
        return self._data_dir
        

def tensorboard_launcher(directory_path):
    from tensorboard import program
    import webbrowser
    # learning visualizer
    tb = program.TensorBoard()
    tb.configure(argv=[None, '--logdir', directory_path])
    url = tb.launch()
    print("[RAISIM_GYM] Tensorboard session created: "+url)
    webbrowser.open_new(url)


def load_param(weight_path, env, actor, critic, optimizer, data_dir):
    if weight_path == "":
        raise Exception("\nCan't find the pre-trained weight, please provide a pre-trained weight with --weight switch\n")
    print("\nRetraining from the checkpoint:", weight_path+"\n")

    # iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
    iteration_number = weight_path.rsplit('\\', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
    print(iteration_number)

    weight_dir = weight_path.rsplit('\\', 1)[0] + '\\'
    print(weight_dir)

    mean_csv_path = weight_dir + 'mean' + iteration_number + '.csv'
    var_csv_path = weight_dir + 'var' + iteration_number + '.csv'
    items_to_save = [weight_path, mean_csv_path, var_csv_path, weight_dir + "cfg.yaml", weight_dir + "Environment.hpp"]

    if items_to_save is not None:
        print(data_dir)
        print("weight_path: " + weight_path.rsplit('\\', 1)[0].rsplit('\\', 1)[1])
        print("items_to_save: ")
        print(items_to_save)
        pretrained_data_dir = data_dir + '/pretrained_' + weight_path.rsplit('\\', 1)[0].rsplit('\\', 1)[1]
        os.makedirs(pretrained_data_dir)
        print("pretrained_data_dir: " + pretrained_data_dir)
        for item_to_save in items_to_save:
        	copy_path = pretrained_data_dir+'/'+item_to_save.rsplit('\\', 1)[1]
        	print("item_to_save: " + item_to_save)
        	print("copy_path: " + copy_path)
        	print("split item_to_save: " + item_to_save.rsplit('\\', 1)[1])
        	copyfile(item_to_save, copy_path)
            # copyfile(item_to_save, pretrained_data_dir+'/'+item_to_save.rsplit('\\', 1)[1])

    # load observation scaling from files of pre-trained model
    # env.load_scaling(weight_dir, iteration_number)
    env.load_scaling(weight_dir, iteration_number,1000000) # may be 10,000,000 or 100,000,000

    # load actor and critic parameters from full checkpoint
    checkpoint = torch.load(weight_path)
    actor.architecture.load_state_dict(checkpoint['actor_architecture_state_dict'])
    actor.distribution.load_state_dict(checkpoint['actor_distribution_state_dict'])
    critic.architecture.load_state_dict(checkpoint['critic_architecture_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
