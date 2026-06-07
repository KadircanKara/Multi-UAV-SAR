import pickle
# from GoogleDriveUpload import authenticate, upload_file, PARENT_FOLDER_ID_DICT

def save_as_pickle(filename:str, data):
    with open(filename, 'wb') as file:
        pickle.dump(data, file)

def load_pickle(path:str):
    with open(path, 'rb') as file:
        data = pickle.load(file)
        return data