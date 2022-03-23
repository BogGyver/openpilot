
#operate with params
OP_PARAMS_PATH = "/data/params/"

def save_bool_param(param_name,param_value):
    try:
        real_param_value = 1 if param_value else 0
        with open(OP_PARAMS_PATH+"/"+param_name, "w") as outfile:
             outfile.write(f'{real_param_value}')
    except IOError:
        print("Failed to save "+param_name+" with value ",param_value)
    

def load_bool_param(param_name,param_def_value):
    try:
        with open(OP_PARAMS_PATH+"/"+param_name, 'r') as f:
            for line in f:
                value_saved = int(line)
        #print("Reading Params ",param_name , "value", value_saved)
        return True if value_saved == 1 else False
    except IOError:
        print("Initializing "+param_name+" with value ",param_def_value)
        save_bool_param(param_name,param_def_value)
        return param_def_value

def save_float_param(param_name,param_value):
    try:
        real_param_value = param_value * 1.0
        with open(OP_PARAMS_PATH+"/"+param_name, "w") as outfile:
             outfile.write(f'{real_param_value}')
    except IOError:
        print("Failed to save "+param_name+" with value ",real_param_value)
    

def load_float_param(param_name,param_def_value):
    try:
        with open(OP_PARAMS_PATH+"/"+param_name, 'r') as f:
            for line in f:
                value_saved = float(line)
        #print("Reading Params ",param_name , "value", value_saved)
        return value_saved * 1.0
    except IOError:
        print("Initializing "+param_name+" with value ",param_def_value*1.0)
        save_float_param(param_name,param_def_value * 1.0)
        return param_def_value * 1.0
        
