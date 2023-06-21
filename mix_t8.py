import spacy
from nltk.corpus import wordnet

nlp = spacy.load("en_core_web_sm")

sentences = "Slice the carrots thinly. Mix the flour and milk in a bowl. Mix the flour and milk in a bowl. Cut the lemon and bread into small pieces. Cook the meat in the oven on medium heat. Put the vegetables on a plate."
nlp_sent = nlp(sentences)


actions = []

def is_similar(word1, word2):
    synsets1 = wordnet.synsets(word1)
    synsets2 = wordnet.synsets(word2)

    for synset1 in synsets1:
        for synset2 in synsets2:
            similarity = synset1.path_similarity(synset2)
            if similarity is not None and similarity > 0.7:
                return True
    return False

def similar_to(word1, word_list):
    for word2 in word_list:
        if is_similar(word1, word2):
            return word2
    return None

food_dict = {
    "carrot": {
        "type": "carrot",
        "location": "counter"
    },
    "milk": {
        "type": "milk",
        "location": "counter"
    },
    "bread": {
        "type": "bread",
        "location": "counter"
    },
    "flour": {
        "type": "flour",
        "location": "counter"
    },
    "lemon": {
        "type": "lemon",
        "location": "counter"
    },
    "meat": {
        "type": "meat",
        "location": "counter"
    },
    "vegetables": {
        "type": "vegetables",
        "location": "counter"
    }
}

class Mix:
    def __init__(self, food_list, place=None):
        self.food = food_list
        self.place = place

    def __str__(self):
        food_str = ", ".join(str(food) for food in self.food)
        return f"Mix: {food_str} in {self.place}" if self.place else f"Mix: {food_str}"

class Cut:
    def __init__(self, food_list, thickness=None):
        self.food = food_list
        self.thickness = thickness
        self.place = "cutting board"

    def __str__(self):
        food_str = ", ".join(str(food) for food in self.food)
        return f"Cut: {food_str} into {self.thickness} on {self.place}"

class Cook:
    def __init__(self, food_list, form=None, heat=None, place=None):
        self.food = food_list
        self.form = form
        self.heat = heat
        self.place = place

    def __str__(self):
        food_str = ", ".join(str(food) for food in self.food)
        return f"Cook: {food_str} in {self.form} on {self.heat} heat using {self.place}"

class Put:
    def __init__(self, food_list, place=None):
        self.food = food_list
        self.place = place

    def __str__(self):
        food_str = ", ".join(str(food) for food in self.food)
        return f"Put: {food_str} on {self.place}"

last_loc = None
last_food = None
for sent in nlp_sent.sents:
    doc = sent

    verb = None
    food = []

    for token in doc:
        if token.pos_ == "VERB":
            verb = token.lemma_
        if token.pos_ == "NOUN" or token.pos_ == "PROPN":
            food.append(token.text)

    action = None
    for action_word in ["mix", "cut", "cook", "put"]:
        if is_similar(verb, action_word):
            action = action_word
            break

    valid_food = []
    for item in food:
        similar_food = similar_to(item, list(food_dict.keys()))
        if similar_food:
            valid_food.append(food_dict[similar_food])
    if valid_food == []:


    if action == "mix":
        place = None
        for token in doc:
            place = similar_to(token.text, ["bowl", "pan", "pot"])
            if place:
                break
        action_instance = Mix(valid_food, place)
    elif action == "cut":
        thickness = None
        for token in doc:
            thickness = similar_to(token.text, ["thin", "thick"])
            if thickness:
                break
        action_instance = Cut(valid_food, thickness)
    elif action == "cook":
        form = None
        heat = None
        for token in doc:
            form = similar_to(token.text, ["oven", "stove"])
            if form:
                break
        if form == "oven":
            place = "oven tray"
        elif form == "stove":
            for token in doc:
                place = similar_to(token.text, ["pot", "pan"])
                if place:
                    break
        for token in doc:
            heat = similar_to(token.text, ["low", "medium", "high"])
            if heat:
                break
        action_instance = Cook(valid_food, form, heat, place)
    elif action == "put":
        place = None
        for token in doc:
            place = similar_to(token.text, ["counter", "plate", "bowl", "pan", "pot", "oven", "cutting board","oven tray"])
            if place:
                break
        action_instance = Put(valid_food, place)
        for food in valid_food:
            food_dict[food_instance["type"]] = action_instance.place
            food["location"] = action_instance.place
    else:
        action_instance = None

    if action_instance is not None:
        foods_not_in_place = []
        for food_instance in valid_food:
            print(type(food))
            if food_instance["location"] != action_instance.place:
                foods_not_in_place.append(food_instance)
                food_dict[food_instance["type"]] = action_instance.place
        if foods_not_in_place != []:
            actions.append(Put(foods_not_in_place,action_instance.place))
        last_loc = action_instance.place
        last_food = valid_food
        actions.append(action_instance)

for action in actions:
    print(action)

print(food_dict)

