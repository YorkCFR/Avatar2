from collections import OrderedDict

class LocalCache():
    def __init__(self):
        self._permanent_entries = set([
            'hi', 'hello', 'good bye', 'goodbye',
            'how are you', 'what is your name', 
            'good morning', 'good afternoon', 'good evening', 'good night', 
        ])
        self._cache = OrderedDict({
            'hi': {"Hi! How can I help you today?"},
            'hello': {"Hi there! How can I assist you today?"},
            'goodbye': {"Goodbye!"},
            'how are you': {"Hi! I'm doing well today. How can I assist you?"},
            'what is your name': {"My name is Mary. How can I assist you today?"},
            'good morning': {"Good morning! How can I assist you today?"},
            'good afternoon': {"Good afternoon! How can I assist you today?"},
            'good evening': {"Good evening! How can I assist you today?"},
            'good night': {"Good evening! How can I assist you today?"},
        })
        self._max_size = 100
