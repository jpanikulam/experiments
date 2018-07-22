import urllib2
import numpy as np
import os


class ParcelMap:
    parcel_map = {}

    @classmethod
    def to_geojson(cls, parcel):
        return {'properties': {
            'lon': parcel['x'],
            'lat': parcel['y'],
        }}

    @classmethod
    def get(cls, parcel_id):
        parcel = cls.parcel_map[parcel_id]
        return cls.to_geojson(parcel)

    @classmethod
    def create(cls, parcel_id, other_data):
        parcel = cls.get(parcel_id)
        parcel['properties'].update(other_data)
        parcel['geometry'] = []
        parcel['type'] = 'Feature'
        return parcel


def maybe_get_parcel_centroids():
    url = "https://data.wprdc.org/dataset/" + \
        "2536e5e2-253b-4c58-969d-687828bb94c6/resource/" + \
        "23267115-177e-4824-89d9-185c7866270d/download/" + \
        "parcelcentroidcoded201802.csv"

    here = os.path.dirname(os.path.realpath(__file__))
    cache_location = os.path.join(here, 'parcel_cache.npy')
    cache_exists = os.path.exists(cache_location) and os.path.isfile(cache_location)

    if cache_exists:
        print('Found cache at {}, using it...'.format(cache_location))
        cache = np.load(cache_location)
    else:
        print('Could not find cache...')
        print('Downloading data...')
        response = urllib2.urlopen(url)
        cache = np.genfromtxt(response, names=True, delimiter=',', dtype=None)
        print('Generating cache at {}'.format(cache_location))
        loc, _ = os.splitext(cache_location)
        np.save(loc, cache)

    pins = cache['PIN']
    return dict(zip(pins, cache))


def init():
    ParcelMap.parcel_map = maybe_get_parcel_centroids()

init()
