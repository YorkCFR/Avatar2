import os
import json
import time
import heapq
import string
import datetime
import rclpy.time

class LocalCache():
    def __init__(self, node, filename=None, max_size=100, root=None):
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
        
        # Logging dir 
        if root is not None:
            self.logging_dir = root + '/logs/'
        else:
            self.logging_dir = "/home/walleed/Avatar2/scenarios/hearing_clinic/logs/"
        
        if filename is not None:
            self._load_cache_fom_file(filename)
        self._initialize_permanent_entries()
            
    def _initialize_permanent_entries(self):
            for key in self.permanent_entries:
                key_normalized = key.lower().strip().translate(str.maketrans('', '', string.punctuation))
                if key_normalized not in self.cache_map:
                    # get the current time in seconds
                    timestamp = time.time()
                    input_time = self._node.get_clock().now().nanoseconds / 1e9 # Initialize the input time to the current time when loading the cache
                    in_cache = True
                    self.cache_map[key_normalized] = (f"Default response for {key_normalized}", timestamp, 0, input_time, in_cache)
                    self._log_responses(key_normalized)
                    heapq.heappush(self.cache, (0, timestamp, key_normalized))
                else:
                    self._node.get_logger().info(f"{self._node.get_name()} Key {key_normalized} already exists in the cache.")
            
    def _load_cache_fom_file(self, filename) -> None:
        self._node.get_logger().info(f"{self._node.get_name()} Loading cache from {filename}")
        if os.path.exists(filename):
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                    for key, (value, timestamp, count, input_time, in_cache) in data.items():
                        key_normalized = key.lower().strip().translate(str.maketrans('', '', string.punctuation))
                        input_time = self._node.get_clock().now().nanoseconds / 1e9 # Initialize the input time to the current time when loading the cache
                        self.cache_map[key_normalized] = (value, timestamp, count, input_time, in_cache)
                        heapq.heappush(self.cache, (count, timestamp, key_normalized))
                        self._log_responses(key_normalized)
                    self._node.get_logger().info(f"{self._node.get_name()} Loaded cache entries from {filename} with {len(data)} entries")
            except Exception as e:
                self._node.get_logger().info(f"{self._node.get_name()} Error loading cache from {filename}: {e}")
        else:
            self._node.get_logger().info(f"{self._node.get_name()} No cache file found at {filename}, starting with an empty cache")
            
    def get(self, query):
        start_time = self._node.get_clock().now().nanoseconds / 1e9
        # Check if the query is in the cache, if so return the response and response time and update the heap
        query_normalized = query.lower().strip().translate(str.maketrans('', '', string.punctuation))
        # Check if the query is in the cache
        if query_normalized in self.cache_map:
            self._node.get_logger().info(f"{self._node.get_name()} Cache hit for {query_normalized}")
            response = self.cache_map[query_normalized][0]
            new_timestamp = ((self._node.get_clock().now().nanoseconds / 1e9) - start_time)
            return response, new_timestamp
        else:
            self._node.get_logger().info(f"{self._node.get_name()} Cache miss for {query_normalized}")
            return None, None
        
    def put(self, query, response, response_time) -> None:
        query_normalized = query.lower().strip().translate(str.maketrans('', '', string.punctuation))
        # If the query is already in the cache, update the value, count, timestamp and input time 
        response_time = float(response_time)
        # if query_normalized in self.cache_map:
        #     self._update_cache(query_normalized, response, response_time)
        # else:
        new_input_time = self._node.get_clock().now().nanoseconds / 1e9
        in_cache = False
        count = 0
        self._node.get_logger().info(f"{self._node.get_name()} Adding to cache for {query_normalized}")
        if len(self.cache) >= self._max_size:
            self._evict_cache()
        self.cache_map[query_normalized] = (response, response_time, count, new_input_time, in_cache)
        # Log the responses
        self._log_responses(query_normalized)
        self._node.get_logger().info(f"{self._node.get_name()} Added {query_normalized} to cache with response time {response_time}")
        heapq.heappush(self.cache, (count, response_time, query_normalized))
            
    def _update_cache(self, query, response, response_time) -> None:
        query_normalized = query.lower().strip().translate(str.maketrans('', '', string.punctuation))
        new_input_time = self._node.get_clock().now().nanoseconds / 1e9
        _, old_timestamp, count, input_time, in_cache = self.cache_map[query_normalized]
        in_cache = True
        new_count = count + 1
        new_timestamp = response_time
        self._node.get_logger().info(f"{self._node.get_name()} Old timestamp: {old_timestamp}, New timestamp: {new_timestamp}")
        self._node.get_logger().info(f"{self._node.get_name()} Old count: {count}, New count: {new_count}")
        self._node.get_logger().info(f"{self._node.get_name()} Old input time: {input_time}, New input time: {new_input_time}")
        self.cache_map[query_normalized] = (response, new_timestamp, new_count, new_input_time, in_cache)
        # Log the responses
        self._log_responses(query_normalized)
        heapq.heappush(self.cache, (new_count, new_timestamp, query))
        
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
                json.dump({key: [value, timestamp, count, input_time, in_cache] for key, (value, timestamp, count, input_time, in_cache) in self.cache_map.items()}, f, indent=4)
                self._node.get_logger().info(f"{self._node.get_name()} Saved cache to {filename}")
        except Exception as e:
            self._node.get_logger().info(f"{self._node.get_name()} Error saving cache to {filename}: {e}")
            
    def _log_responses(self, query) -> None:
        if query in self.cache_map:
            key = query
            (value, timestamp, count, input_time, in_cache) = self.cache_map[query]
            self._node.get_logger().info(f"{self._node.get_name()} Logging entry: {key} -> {value}, {timestamp}, {count}, {input_time}, {in_cache}")
            # Get the entry time of the query
            entry_time = input_time
            # Get the timestamp of the query 
            query_time = timestamp
            # Get the count of the query
            query_count = count
            # Get the response of the query
            query_response = value
            
            # Current time of logging
            current_time = self._node.get_clock().now().nanoseconds / 1e9
            # Get the time difference between the current time and the entry time
            time_diff = current_time - entry_time
            
            # Log the query, response, timestamp, count, entry time, current time and time difference to a csv file
            log_filename = self.logging_dir + datetime.datetime.fromtimestamp(current_time).strftime('%Y-%m-%d') + "_log.csv"
            if os.path.exists(log_filename):
                with open(log_filename, 'a') as f:
                    f.write(f"{key}, {query_response}, {query_time}, {query_count}, {datetime.datetime.fromtimestamp(entry_time)}, {current_time}, {time_diff}, {in_cache} \n")
            else:
                with open(log_filename, 'w') as f:
                    f.write("query, response, timestamp (time taken to generate/update the response), count, entry time, current time of logging, time difference\n")
                    f.write(f"{key}, {query_response}, {query_time}, {query_count}, {entry_time}, {current_time}, {time_diff}, {in_cache}\n")
            self._node.get_logger().info(f"Entry logged to {log_filename}")
