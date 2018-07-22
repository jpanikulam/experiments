'''
psycopg2
sqlalchemy
'''

from database_schema import Property

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

import json
from get_properties import ParcelMap

db_string = "postgres://jpanikulam:uf8cjea89c*(HH@re.benkroop.com:5435/bkroop"
db = create_engine(db_string)
parcel_id = '0006K0035%'
Session = sessionmaker(db)
session = Session()
# q = session.query(Property).filter(Property.parcel_id.like(parcel_id))
# q = session.query(Property).filter(Property.owner_name.contains('DIESCH'))
q = session.query(Property).filter(Property.property_address.contains('ST'))

geojson = {
    'type': 'FeatureCollection',
    'features': [],
}


def generate_geojson(query_results, parcels_to_people):
    assert isinstance(query_results, list)
    for parcel in query_results:
        owner = parcel.owner_name
        price = parcel.sale_price
        prev_price = parcel.previous_sale_price
        extra_data = {}
        extra_data['price'] = parcel.sale_price
        extra_data['prev_price'] = parcel.previous_sale_price

        extra_data['owner'] = owner
        if parcel.parcel_id in parcels_to_people.keys():
            extra_data['job'] = parcels_to_people[parcel.parcel_id]['company']
            print "HAVE"
        else:
            continue

        if parcel.sale_date is not None:
            extra_data['sale_date'] = parcel.sale_date.replace('-', '/')
        if parcel.parcel_id in ParcelMap.parcel_map:
            geo = ParcelMap.create(parcel.parcel_id, extra_data)
        else:
            continue

        print("{}: {} {}".format(
            owner,
            parcel.property_house_num,
            parcel.property_address,
        ))

        print("  Most Recent : (${}; {})".format(price, parcel.sale_date))
        print("  Previous    : (${}; {})".format(prev_price, parcel.previous_sale_date))
        print("  Previous 2  : (${}; {})".format(parcel.previous_sale_price2, parcel.previous_sale_date2))

        geojson['features'].append(geo)
        print '---'
        print geo
    return geojson


def main():
    parcels_to_people = json.load(open('parcels_to_people.json', 'r'))
    output_json = generate_geojson(q.all(), parcels_to_people)
    json.dump(output_json, open('foopa.geojson', 'w'))

if __name__ == '__main__':
    main()
