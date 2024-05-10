
def get_queue_data(queue):
    if not queue.empty():
        return queue.get()
    else:
        return None
    
def normalize(min, max, value):
    return (value - min) / (max - min)