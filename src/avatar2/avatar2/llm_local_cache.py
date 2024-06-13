import os
import json
import time
import heapq
import string
from collections import OrderedDict

class LocalCache():
    def __init__(self, node, filename=None, max_size=100):
        self._max_size = max_size
        self._node = node
        self._cache = {}
        self._heap = []
        self._heap_map = {}
        self._counter = 0  # Timestamp counter to keep track of the order
        self._node.get_logger().info(f"{self._node.get_name()} LocalCache alive")
        
        self._permanent_entries = set([
            'hi', 'hello', 'good bye', 'goodbye',
            'how are you', 'what is your name', 
            'good morning', 'good afternoon', 'good evening', 'good night', 
        ])
        
        if filename:
            self._load_cache_fom_file(filename)
        else:
            self._initialize_permanent_entries()
            
    def _initialize_permanent_entries(self):
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
            
    def _load_cache_fom_file(self, filename) -> None:
        if os.path.exists(filename):
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                    for key, value in data.items():
                        self._cache[key] = value
                        self._add_to_heap(key)
                    self._node.get_logger().info(f"{self._node.get_name()} Loaded cache entries from {filename} with {len(data)} entries")
            except Exception as e:
                self._node.get_logger().info(f"{self._node.get_name()} Error loading cache from {filename}: {e}")
        else:
            self._node.get_logger().info(f"{self._node.get_name()} No cache file found at {filename}")
            
    def query_cache(self, query):
        # Check if the query is in the cache, if so return the response and response time and update the heap
        query_normalized = query.lower().strip().translate(str.maketrans('', '', string.punctuation))
        # Check if the query is in the cache
        if query_normalized in self._cache:
            self._node.get_logger().info(f"{self._node.get_name()} Cache hit for {query_normalized}")
            start_time = time.time()
            response, response_time = self._cache[query_normalized]
            time_taken = time.time() - start_time
            if time_taken < 0.2:
                self._node.get_logger().info(f"{self._node.get_name()} Cache response time for {query_normalized} is {time_taken}")
                response_time = 0.1 # Minimum response time for cache hit
            self._cache[query_normalized] = (response, response_time)
            self._update_heap(query_normalized)
            return response, response_time
        return None, None
        
    def add_to_cache(self, query, response, response_time) -> None:
        # Add the query to the cache and update the heap
        query_normalized = query.lower().strip().translate(str.maketrans('', '', string.punctuation))
        self._node.get_logger().info(f"{self._node.get_name()} Adding {query_normalized} to cache")
        if len(self._cache) >= self._max_size:
            self._node.get_logger().info(f"{self._node.get_name()} Cache full, purging oldest entry")
            self._evict_cache()
        self._cache[query_normalized] = (response, response_time)
        self._update_heap(query_normalized)
    
    def _update_heap(self, query):
        if query in self._heap_map:
            entry = self._heap_map[query]
            self._heap.remove(entry)
            heapq.heapify(self._heap)
        self._add_to_heap(query)
            
    def _add_to_heap(self, query):
        self._counter += 1
        entry = (self._counter, query)
        heapq.heappush(self._heap, entry)
        self._heap_map[query] = entry
        
    def _evict_cache(self):
        while self._heap:
            timestamp, oldest = self._heap[0]
            if oldest not in self._permanent_entries:
                del self._cache[oldest]
                del self._heap_map[oldest]
                self._node.get_logger().info(f"{self._node.get_name()} Evicted {oldest} from cache")
                break
    def save_cache_to_file(self, filename) -> None:
        try:
            with open(filename, 'w') as f:
                # Serialize the cache to a file including the timestamp to JSON
                cache_data = {key: list(value) for key, value in self._cache.items()}
                json.dump(cache_data, f)
                self._node.get_logger().info(f"{self._node.get_name()} Saved cache to {filename}")
        except Exception as e:
            self._node.get_logger().info(f"{self._node.get_name()} Error saving cache to {filename}: {e}")