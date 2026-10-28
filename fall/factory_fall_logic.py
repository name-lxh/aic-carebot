# -*- coding: utf-8 -*-
"""Temporal confirmation for factory fall/no fall detections, Python 2/3."""
import math


class FactoryFallEvidence(object):
    def __init__(self):
        self.last_time = None
        self.center = None
        self.since = None

    def reset(self):
        self.__init__()

    def update(self, now, people):
        if self.last_time is not None and (now <= self.last_time or now-self.last_time > 1.5):
            self.reset()
        self.last_time = now
        if len(people) != 1:
            self.since = None
            self.center = None
            return 'WAIT_ONE_FACTORY_PERSON'
        person = people[0]
        score = person.get('score', 0.)
        if not isinstance(score, (float,int)) or math.isnan(score) or math.isinf(score) or score < .5:
            self.since = None
            self.center = None
            return 'WAIT_FACTORY_CONFIDENCE'
        center = person['center']
        if self.center is not None and math.hypot(center[0]-self.center[0],center[1]-self.center[1]) > .30:
            self.since = None
        self.center = center
        if person.get('label') != 'fall':
            self.since = None
            return 'WAIT_FACTORY_FALL'
        if self.since is None:
            self.since = now
        return 'FALL_CONFIRMED' if now-self.since >= 1. else 'CONFIRM_FACTORY_FALL'
