import os
import json
import time
import heapq
import string

class LocalCache():
    def __init__(self, node, filename=None, max_size=100):
        self._max_size = max_size
        self._node = node
        self.cache = []
        self.cache_map = {}
        self.permanent_entries = {
            'hi', 'hello', 'what is your name', 'how are you', 'tell me some information about the clinic',
            'what kind of services are available at the clinic', 'who is the owner of the clinic', 'what can you tell me about penny', 'what does cyanide taste like',
            'how many different types of hairing aids are there', 'what is the most common type of hearing aid', 'what is the most expensive hearing aid',
            'goodbye'
        }
        self._node.get_logger().info(f"{self._node.get_name()} LocalCache alive!")
        
        if filename is not None:
            self._load_cache_fom_file(filename)
        self._initialize_permanent_entries()
            
    def _initialize_permanent_entries(self):
            for key in self.permanent_entries:
                key_normalized = key.lower().strip().translate(str.maketrans('', '', string.punctuation))
                if key_normalized not in self.cache_map:
                    timestamp = self._current_time()
                    self.cache_map[key_normalized] = (f"Default response for {key_normalized}", timestamp, 0)
                    heapq.heappush(self.cache, (0, timestamp, key_normalized))
                else:
                    self._node.get_logger().info(f"{self._node.get_name()} Key {key_normalized} already exists in the cache.")
            
    def _load_cache_fom_file(self, filename) -> None:
        self._node.get_logger().info(f"{self._node.get_name()} Loading cache from {filename}")
        if os.path.exists(filename):
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                    for key, (value, timestamp, count) in data.items():
                        key_normalized = key.lower().strip().translate(str.maketrans('', '', string.punctuation))
                        self.cache_map[key_normalized] = (value, timestamp, count)
                        heapq.heappush(self.cache, (count, timestamp, key_normalized))
                    self._node.get_logger().info(f"{self._node.get_name()} Loaded cache entries from {filename} with {len(data)} entries")
            except Exception as e:
                self._node.get_logger().info(f"{self._node.get_name()} Error loading cache from {filename}: {e}")
        else:
            self._node.get_logger().info(f"{self._node.get_name()} No cache file found at {filename}, starting with an empty cache")
            
    def get(self, query):
        start_time = self._node.get_clock().now()
        # Check if the query is in the cache, if so return the response and response time and update the heap
        query_normalized = query.lower().strip().translate(str.maketrans('', '', string.punctuation))
        # Check if the query is in the cache
        if query_normalized in self.cache_map:
            self._node.get_logger().info(f"{self._node.get_name()} Cache hit for {query_normalized}")
            response, timestamp, count = self.cache_map[query_normalized]
            # Update the timestamp to the current time and increment count
            new_timestamp = (self._node.get_clock().now() - start_time).nanoseconds / 1e9
            new_count = count + 1
            self.cache_map[query_normalized] = (response, new_timestamp, new_count)
            # Update the heap with the new timestamp and count
            self._node.get_logger().info(f"{self._node.get_name()} Updating cache for {query_normalized} with new timestamp {new_timestamp}")
            heapq.heappush(self.cache, (new_count, new_timestamp, query_normalized))
            return response, new_timestamp
        else:
            self._node.get_logger().info(f"{self._node.get_name()} Cache miss for {query_normalized}")
            return None, None
        
    def put(self, query, response, response_time) -> None:
        query_normalized = query.lower().strip().translate(str.maketrans('', '', string.punctuation))
        # If the query is already in the cache, update the value and timestamp
        response_time = float(response_time)
        if query_normalized in self.cache_map:
            self._node.get_logger().info(f"{self._node.get_name()} Updating cache for {query_normalized}")
            _, old_timestamp, count = self.cache_map[query_normalized]
            new_timestamp = response_time
            new_count = count + 1
            self._node.get_logger().info(f"{self._node.get_name()} Old timestamp: {old_timestamp}, New timestamp: {new_timestamp}")
            self.cache_map[query_normalized] = (response, new_timestamp, new_count)
            heapq.heappush(self.cache, (new_count, new_timestamp, query_normalized))
        else:
            self._node.get_logger().info(f"{self._node.get_name()} Adding to cache for {query_normalized}")
            if len(self.cache) >= self._max_size:
                self._evict_cache()
            self.cache_map[query_normalized] = (response, response_time, 0)
            self._node.get_logger().info(f"{self._node.get_name()} Added {query_normalized} to cache with response time {response_time}")
            heapq.heappush(self.cache, (0, response_time, query_normalized))
    
    # def _update_heap(self, query):
    #     if query in self._heap_map:
    #         entry = self._heap_map[query]
    #         self._heap.remove(entry)
    #         heapq.heapify(self._heap)
    #     self._add_to_heap(query)
            
    # def _add_to_heap(self, query):
    #     self._counter += 1
    #     entry = (self._counter, query)
    #     heapq.heappush(self._heap, entry)
    #     self._heap_map[query] = entry
        
    def _evict_cache(self):
        while self._heap:
            count, timestamp, oldest = heapq.heappop(self.cache)
            if oldest not in self.permanent_entries and oldest in self.cache_map and self.cache_map[oldest][1] == timestamp:
                del self.cache_[oldest]
                self._node.get_logger().info(f"{self._node.get_name()} Evicted {oldest} from cache")
                break
            
    def save_cache_to_file(self, filename) -> None:
        try:
            with open(filename, 'w') as f:
                json.dump({key: [value, timestamp, count] for key, (value, timestamp, count) in self.cache_map.items()}, f, indent=4)
                self._node.get_logger().info(f"{self._node.get_name()} Saved cache to {filename}")
        except Exception as e:
            self._node.get_logger().info(f"{self._node.get_name()} Error saving cache to {filename}: {e}")