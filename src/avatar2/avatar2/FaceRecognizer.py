from pathlib import Path
import face_recognition
import pickle
from collections import Counter
import json
import os

# Author: Baran Parsai
# Email: parsaibaran@gmail.com
# Version: 1.1
# Date: 06/21

class FaceRecognizer:
    DEFAULT_ENCODINGS_PATH = Path(os.path.join("/home/baranparsai/Documents/Avatar2/scenarios/hearing_clinic/faces", "faces.pkl"))
    PEOPLE_INFO_PATH = Path(os.path.join("/home/baranparsai/Documents/Avatar2/scenarios/hearing_clinic/faces", "faces.json"))
    
    def __init__(self, encodings=DEFAULT_ENCODINGS_PATH, database = PEOPLE_INFO_PATH, debug = False):
        self.people_info = []
        self._encodings_location = encodings
        self.load_encodings()
        self._debug = debug
    
        with open(database) as f:
            self._database = json.load(f)
        self._print(self._database)

    def _print(self,s):
        if self._debug:
            self._print(s)

    def load_encodings(self):
        """Load face encodings from a pickle file."""
        with self._encodings_location.open(mode="rb") as f:
            self._loaded_encodings = pickle.load(f)
    
    def _create_directories(self):
        """Create necessary directories if they don't exist."""
        Path("training").mkdir(exist_ok=True)
        Path("output").mkdir(exist_ok=True)
        Path("validation").mkdir(exist_ok=True)
    
    @staticmethod
    def encode_known_faces(data_loc: str = "training", model: str = "hog", face_db = DEFAULT_ENCODINGS_PATH, face_info = PEOPLE_INFO_PATH) -> None:
        """Encode known faces into a .pkl file and store individual data into a .JSON file."""
        person_id = 0
        people_info = []
        encodings = []
        json_out = []
        print(data_loc)

        for person in Path(data_loc).glob("*"):
            print(person)
            first_name, last_name = person.name.split("_")
            person_info = {'first_name': first_name, 'last_name': last_name, 'ID': person_id, 'role': 'unknown'}
            json_file = []

            for z in Path(person).glob("*.json"):
                json_file.append(z)
            print(json_file)

            if len(json_file) != 1 :
                person_json_path = os.path.join(person, f"{first_name}_{last_name}.json")
                with open(person_json_path, 'w') as person_json_file:
                    json.dump(person_info, person_json_file, indent=4)
                return 
                
            with open(json_file[0]) as f:
                person_info.update(json.load(f))
                person_info['ID'] = person_id
            json_out.append(person_info)

            for filepath in Path(person).glob("*"):
                if filepath.suffix == ".json":
                    continue
                
                image = face_recognition.load_image_file(filepath)
                face_locations = face_recognition.face_locations(image, model=model)
                face_encodings = face_recognition.face_encodings(image, face_locations)

                if len(face_encodings) != 1:
                    continue
                encodings.append(face_encodings[0])
                people_info.append(person_id)
            person_id += 1

        name_encodings = {'names': people_info, 'encodings': encodings}
        with open(face_db, 'wb') as pkl_file:
            pickle.dump(name_encodings, pkl_file)

        with open(face_info, 'w') as f:
            json.dump(json_out, f, indent=4)


    def recognize_faces(self, input_image, model: str = "hog", encodings_location: Path = DEFAULT_ENCODINGS_PATH) -> None:
        """Identify the largest face in the input image,
        return its location, name, and center coordinates."""

        input_face_locations = face_recognition.face_locations(input_image, model=model)
        input_face_encodings = face_recognition.face_encodings(input_image, input_face_locations)

        largest_face_area = 0
        largest_face_location = None
        middle_row = 0.0
        middle_col = 0.0

        for bounding_box, unknown_encoding in zip(input_face_locations, input_face_encodings):
            top, right, bottom, left = bounding_box
            face_area = (right - left) * (bottom - top)
            middle_row = (right + left) / 2
            middle_col = (bottom + top) / 2

            if face_area > largest_face_area:
                largest_face_area = face_area
                largest_face_location = bounding_box

        self._print(f"largest box is {largest_face_area}")
        id = -1
        name = {'name': 'unknown', 'ID': -1, 'role': 'unknown'}

        if largest_face_area > 0:
            id = self._recognize_face(unknown_encoding, self._loaded_encodings)
            self._print(f"the id is {id}")

            if id >= 0:
                name = self._database[id]

        return largest_face_location, name, middle_row, middle_col


    def _recognize_face(self, unknown_encoding, loaded_encodings):
        """Recognize the face by comparing its encoding with loaded encodings.
        return the face with hisghest score or -1"""

        boolean_matches = face_recognition.compare_faces(loaded_encodings["encodings"], unknown_encoding)
        votes = Counter(
            name
            for match, name in zip(boolean_matches, loaded_encodings["names"])
            if match
        )
        
        if votes:
            all_faces = Counter(loaded_encodings["names"])
            self._print(f"all_faces {all_faces}")
            
            all_faces_l = list(all_faces)
            self._print(f"all_faces_l {all_faces_l}")

            best_face = all_faces_l[0]
            best_count = -1

            for face in all_faces_l:
                self._print(f"working out face {face} votes {votes[face]} total_faces {all_faces[face]} best_face {best_face}")
                score = float(votes[face]) / float(all_faces[face])

                if score > best_count:
                    best_face = face
                    best_count = score
            return best_face
        
        return -1
    
