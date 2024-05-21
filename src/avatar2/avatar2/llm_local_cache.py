import os
import json
import string
from collections import OrderedDict

class LocalCache():
    def __init__(self, node, filename=None):
        self._max_size = 100
        self._node = node
        self._node.get_logger().info(f"{self._node.get_name()} LocalCache alive")
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
        
        if filename:
            self._load_cache_fom_file(filename)
            
    def _load_cache_fom_file(self, filename) -> None:
        if os.path.exists(filename):
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                    for key, value in data.items():
                        self._cache[key] = value
                    self._node.get_logger().info(f"{self._node.get_name()} Loaded cache entries from {filename} with {len(data)} entries")
            except Exception as e:
                self._node.get_logger().info(f"{self._node.get_name()} Error loading cache from {filename}: {e}")
        else:
            self._node.get_logger().info(f"{self._node.get_name()} No cache file found at {filename}")
    
    def _evict_cache(self):
        self._node.get_logger().info(f"{self._node.get_name()} The current cache contains {self._cache.keys()}")
        oldest = ((key for key in self._cache if key not in self._permanent_entries)).__next__()
        self._node.get_logger().info(f"{self._node.get_name()} Evicting {oldest}")
        if oldest:
            self._cache.pop(oldest)
    
    def _process_query(self, query):
        query_normalized = query.lower().strip().translate(str.maketrans('', '', string.punctuation))
        self._node.get_logger().info(f"{self._node.get_name()} Normalized query: {query_normalized}")
        
        if query_normalized in self._cache:
            self._node.get_logger().info(f"{self._node.get_name()} Cache hit for {query_normalized}")
            self._cache.move_to_end(query_normalized)
            return self._cache[query_normalized]
        
        self._node.get_logger().info(f"{self._node.get_name()} Cache miss for {query_normalized}")
        response = None
        
    def _update_cache(self, query_normalized, response) -> None:
        if len(self._cache) >= self._max_size:
            self._node.get_logger().info(f"{self._node.get_name()} Cache full, purging oldest entry")
            self._evict_cache()
        self._cache[query_normalized] = response