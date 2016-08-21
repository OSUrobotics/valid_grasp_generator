#!/usr/bin/env python

import numpy as np

def get_similar_contact_score(a,b):
    if len(a) < len(b):
        prime = a
        target = b
    else:
        prime = b
        target = a

    no_of_overall_contact = len(prime)
    boolean_array = np.in1d(prime,target)
    no_of_common_contacts = len(np.where(boolean_array)[0])
    score = float(no_of_common_contacts)/float(no_of_overall_contact)

    return score*100

if __name__ == "__main__":
    a = np.array(['a','b','c','d','e','g'])
    b = np.array(['a','b','c'])

    score = get_similar_contact_score(a,b)
    print score
