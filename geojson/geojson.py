import json
import time


def load_first_n_features(txt, n):
    print 'Parsing...'
    tic = time.time()
    rdata = json.loads(txt)
    toc = time.time()

    print "Took {}s".format(toc - tic)
    print 'Done parsing...'

    new_json = {
        u'features': rdata['features'][:n],
        u'type': u'FeatureCollection',
    }

    for ftr in new_json['features']:
        acreage = ftr['properties']['CALCACREAGE']
        ftr['properties'] = {'PIN': ftr['properties']['PIN']}
        ftr['properties']['altitude'] = acreage
        # ftr['geometry']['coordinates'] = [ftr['geometry']['coordinates'][0]]
        ftr['properties']['lon'] = ftr['geometry']['coordinates'][0][0][0]
        ftr['properties']['lat'] = ftr['geometry']['coordinates'][0][0][1]
        ftr['geometry'] = []

    with open('reduced-huge.json', 'w') as foop:
        foop.write(json.dumps(new_json))


def main():
    # file = "/home/jacob/Downloads/doop.geojsoin"
    # file = "/home/jacob/Downloads/Allegheny_County_Parcel_Boundaries.geojson"
    file = "/home/jacob/Downloads/Allegheny_County_Parcel_Boundaries.geojson"
    # file = "reduced.json"

    with open(file) as f:
        txt = f.read()
        print 'Finished reading'
        load_first_n_features(txt, 1000000)
        print 'Finished writing'


if __name__ == '__main__':
    main()
