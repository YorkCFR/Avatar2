import cv2
import sys
from pathlib import Path
import face_recognition
import pickle
from collections import Counter
from PIL import Image, ImageDraw
import json
import os

# Author: Baran Parsai
# Email: parsaibaran@gmail.com
# Version: 5
# Date: 06/13

class FaceRecognizer:
    DEFAULT_ENCODINGS_PATH = Path(os.path.join("/home/baranparsai/Documents/Avatar2/people", "faces.pkl"))
    PEOPLE_INFO_PATH = Path(os.path.join("/home/baranparsai/Documents/Avatar2/people", "faces.json"))
    
    def __init__(self, encodings=DEFAULT_ENCODINGS_PATH, database = PEOPLE_INFO_PATH):
        self.people_info = []
        self._encodings_location = encodings
        self.load_encodings()
        with open(database) as f:
            self._database = json.load(f)
        print(self._database)

    def load_encodings(self):
        with self._encodings_location.open(mode="rb") as f:
            self._loaded_encodings = pickle.load(f)
    
    def _create_directories(self):
        """Create necessary directories if they don't exist."""
        Path("training").mkdir(exist_ok=True)
        Path("output").mkdir(exist_ok=True)
        Path("validation").mkdir(exist_ok=True)
    
    @staticmethod
    def encode_known_faces(data_loc: str = "training", model: str = "hog", face_db: str = "faces.pkl", face_info: str = "faces.json") -> None:
        """Encode faces into a pkl and json representation"""
        person_id = 0
        people_info = []
        encodings = []
        json_out = []
        print(data_loc)

        for person in Path(data_loc).glob("*"):
            print(person)
            person_info = {'first_name': 'unknown', 'last_name': 'unknown', 'ID': person_id, 'role': 'unknown'}
            
            json_file = [] 
            for z in Path(person).glob("*.json"):
                json_file.append(z)
            print(json_file)
            if len(json_file) != 1 :
                print("Too many json files")
                print(json_file)
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
        """Recognize faces in an image."""

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

        print(f"largest box is {largest_face_area}")
        id = -1
        name = {'name': 'unknown', 'ID': -1, 'role': 'unknown'}
        if largest_face_area > 0:
            id = self._recognize_face(unknown_encoding, self._loaded_encodings)
            print(f"the id is {id}")
            if id >= 0:
                name = self._database[id]

        return largest_face_location, name, middle_row, middle_col


    def _display_face(self, draw, bounding_box, name):
        top, right, bottom, left = bounding_box
        draw.rectangle(((left, top), (right, bottom)), outline="blue")
        text_left, text_top, text_right, text_bottom = draw.textbbox((left, bottom), name)
        draw.rectangle(((text_left, text_top), (text_right, text_bottom)), fill="blue", outline="blue")
        draw.text((text_left, text_top), name, fill="white")


    def _recognize_face(self, unknown_encoding, loaded_encodings):
        """return the face with most hisghest score or -1"""
        boolean_matches = face_recognition.compare_faces(loaded_encodings["encodings"], unknown_encoding)
        votes = Counter(
            name
            for match, name in zip(boolean_matches, loaded_encodings["names"])
            if match
        )
        if votes:
            all_faces = Counter(loaded_encodings["names"])
            print(f"all_faces {all_faces}")
            all_faces_l = list(all_faces)
            print(f"all_faces_l {all_faces_l}")

            best_face = all_faces_l[0]
            best_count = -1
            for face in all_faces_l:
                print(f"working out face {face} votes {votes[face]} total_faces {all_faces[face]} best_face {best_face}")
                score = float(votes[face]) / float(all_faces[face])
                if score > best_count:
                    best_face = face
                    best_count = score
            return best_face

        #print(f'encodings {loaded_encodings["encodings"]}')
        #if votes:
        #    return votes.most_common(1)[0][0]
        return -1
            

    def validate(self, model: str = "hog"):
        """Validate faces in the validation directory."""
        for filepath in Path("validation").rglob("*"):
            if filepath.is_file():
                self.recognize_faces(image_location=str(filepath.absolute()), model=model)
                

