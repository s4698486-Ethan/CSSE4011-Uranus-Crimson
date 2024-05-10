
def get_queue_data(queue):
    if not queue.empty():
        return queue.get()
    else:
        return None
    
def normalize(new_min, new_max, old_min, old_max ,value):
    return ((value - old_min) / (old_max - old_min)) * new_max