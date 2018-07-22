from sqlalchemy import Column, Integer, String, Boolean, ForeignKey
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class Property(Base):
    __tablename__ = 'property'

    id = Column(Integer, primary_key=True)

    # General Information page
    # http://www2.county.allegheny.pa.us/RealEstate/GeneralInfo.aspx?ParcelID=0049F00229000000
    parcel_id = Column(String)
    school_district_code = Column(String)
    school_district_name = Column(String)
    neighborhood_code = Column(String)
    neighborhood_description = Column(String)
    tax_code = Column(String)
    owner_code = Column(String)
    property_class = Column(String)
    property_class_description = Column(String)
    recording_date = Column(String) # "YYYY-MM-DD"
    use_code = Column(String)
    sale_date = Column(String) # "YYYY-MM-DD"
    homestead = Column(Boolean)
    sale_price = Column(String)
    farmstead = Column(Boolean)
    deed_book = Column(String)
    deed_page = Column(String)
    other_abatement = Column(Boolean)
    lot_area_rea_sq_ft = Column(Integer)
    owner_mailing_address = Column(String)
    owner_name = Column(String)

    # Temporary fields. Should be moved to other tables in the future.
    previous_sale_date = Column(String)
    previous_sale_price = Column(String)
    previous_sale_date2 = Column(String)
    previous_sale_price2 = Column(String)
    as_of_date = Column(String)
    basement_code = Column(String)
    basement_description = Column(String)
    bedrooms = Column(String)
    card_number = Column(String)
    cdu = Column(String)
    cdu_description = Column(String)
    change_notice_address1 = Column(String)
    change_notice_address2 = Column(String)
    change_notice_address3 = Column(String)
    change_notice_address4 = Column(String)
    clean_green = Column(String)
    condition = Column(String)
    condition_description = Column(String)
    county_building = Column(String)
    county_exempt_building = Column(String)
    county_land = Column(String)
    county_total = Column(String)
    fair_market_building = Column(String)
    fair_market_land = Column(String)
    fair_market_total = Column(String)
    finished_living_area = Column(String)
    fireplaces = Column(String)
    full_baths = Column(String)
    grade = Column(String)
    grade_description = Column(String)
    half_baths = Column(String)
    heating_cooling = Column(String)
    heating_cooling_description = Column(String)
    legal_address_1 = Column(String)
    legal_address_2 = Column(String)
    legal_address_3 = Column(String)
    local_building = Column(String)
    local_land = Column(String)
    local_total = Column(String)
    muni_code = Column(String)
    muni_description = Column(String)
    owner_description = Column(String)
    property_address = Column(String)
    property_city = Column(String)
    property_state = Column(String)
    property_fraction = Column(String)
    property_house_num = Column(String)
    property_unit = Column(String)
    property_zip = Column(String)
    record_date = Column(String)
    roof_code = Column(String)
    roof_description = Column(String)
    sale_code = Column(String)
    sale_date = Column(String)
    sale_description = Column(String)
    sale_price = Column(String)
    stories = Column(String)
    style_code = Column(String)
    style_description = Column(String)
    tax_code = Column(String)
    tax_description = Column(String)
    tax_subcode = Column(String)
    tax_subcode_description = Column(String)
    tax_year = Column(String)
    use_code = Column(String)
    use_description = Column(String)
    year_built = Column(String)
    exterior_finish_description = Column(String)

    @classmethod
    def from_dictionary(cls, dictionary_record):
        p = cls()
        # p.other_abatement = dictionary_record.get('ABATEMENTFLAG', None)
        # p. = dictionary_record.get('ALT_ID', None)
        p.as_of_date = dictionary_record.get('ASOFDATE', None)
        p.basement = dictionary_record.get('BASEMENT', None)
        p.basement_description = dictionary_record.get('BASEMENTDESC', None)
        p.bedrooms = dictionary_record.get('BEDROOMS', None)
        p.basement_garage = dictionary_record.get('BSMTGARAGE', None)
        p.card_number = dictionary_record.get('CARDNUMBER', None)
        p.cdu = dictionary_record.get('CDU', None)
        p.cdu_description = dictionary_record.get('CDUDESC', None)
        p.change_notice_address1 = dictionary_record.get('CHANGENOTICEADDRESS1', None)
        p.change_notice_address2 = dictionary_record.get('CHANGENOTICEADDRESS2', None)
        p.change_notice_address3 = dictionary_record.get('CHANGENOTICEADDRESS3', None)
        p.change_notice_address4 = dictionary_record.get('CHANGENOTICEADDRESS4', None)
        p.property_class = dictionary_record.get('CLASS', None)
        p.property_class_description = dictionary_record.get('CLASSDESC', None)
        p.clean_green = dictionary_record.get('CLEANGREEN', None)
        p.condition = dictionary_record.get('CONDITION', None)
        p.condition_description = dictionary_record.get('CONDITIONDESC', None)
        p.county_building = dictionary_record.get('COUNTYBUILDING', None)
        p.county_exempt_building = dictionary_record.get('COUNTYEXEMPTBLDG', None)
        p.county_land = dictionary_record.get('COUNTYLAND', None)
        p.county_total = dictionary_record.get('COUNTYTOTAL', None)
        p.deed_book = dictionary_record.get('DEEDBOOK', None)
        p.deed_page = dictionary_record.get('DEEDPAGE', None)
        p.exterior_finish = dictionary_record.get('EXTERIORFINISH', None)
        p.exterior_finish_description = dictionary_record.get('EXTFINISH_DESC', None)
        p.fair_market_building = dictionary_record.get('FAIRMARKETBUILDING', None)
        p.fair_market_land = dictionary_record.get('FAIRMARKETLAND', None)
        p.fair_market_total = dictionary_record.get('FAIRMARKETTOTAL', None)
        # p.farmstead = dictionary_record.get('FARMSTEADFLAG', None)
        p.finished_living_area = dictionary_record.get('FINISHEDLIVINGAREA', None)
        p.fireplaces = dictionary_record.get('FIREPLACES', None)
        p.full_baths = dictionary_record.get('FULLBATHS', None)
        p.grade = dictionary_record.get('GRADE', None)
        p.grade_description = dictionary_record.get('GRADEDESC', None)
        p.half_baths = dictionary_record.get('HALFBATHS', None)
        p.heating_cooling = dictionary_record.get('HEATINGCOOLING', None)
        p.heating_cooling_description = dictionary_record.get('HEATINGCOOLINGDESC', None)
        # p.homestead = dictionary_record.get('HOMESTEADFLAG', None)
        p.legal_address_1 = dictionary_record.get('LEGAL1', None)
        p.legal_address_2 = dictionary_record.get('LEGAL2', None)
        p.legal_address_3 = dictionary_record.get('LEGAL3', None)
        p.local_building = dictionary_record.get('LOCALBUILDING', None)
        p.local_land = dictionary_record.get('LOCALLAND', None)
        p.local_total = dictionary_record.get('LOCALTOTAL', None)
        try:
            p.lot_area_rea_sq_ft = int(dictionary_record.get('LOTAREA', None))
        except Exception as e:
            print("Could not convert lot area to string")
            print(e)
        p.muni_code = dictionary_record.get('MUNICODE', None)
        p.muni_description = dictionary_record.get('MUNIDESC', None)
        p.neighborhood_code = dictionary_record.get('NEIGHCODE', None)
        p.neighborhood_description = dictionary_record.get('NEIGHDESC', None)
        p.owner_code = dictionary_record.get('OWNERCODE', None)
        p.owner_description = dictionary_record.get('OWNERDESC', None)
        p.parcel_id = dictionary_record.get('PARID', None)
        p.previous_sale_date = dictionary_record.get('PREVSALEDATE', None)
        p.previous_sale_date2 = dictionary_record.get('PREVSALEDATE2', None)
        p.previous_sale_price = dictionary_record.get('PREVSALEPRICE', None)
        p.previous_sale_price2 = dictionary_record.get('PREVSALEPRICE2', None)
        p.property_address = dictionary_record.get('PROPERTYADDRESS', None)
        p.property_city = dictionary_record.get('PROPERTYCITY', None)
        p.property_fraction = dictionary_record.get('PROPERTYFRACTION', None)
        p.property_house_num = dictionary_record.get('PROPERTYHOUSENUM', None)
        p.property_state = dictionary_record.get('PROPERTYSTATE', None)
        p.property_unit = dictionary_record.get('PROPERTYUNIT', None)
        p.property_zip = dictionary_record.get('PROPERTYZIP', None)
        p.record_date = dictionary_record.get('RECORDDATE', None)
        p.roof_type = dictionary_record.get('ROOF', None)
        p.roof_description = dictionary_record.get('ROOFDESC', None)
        p.sale_code = dictionary_record.get('SALECODE', None)
        p.sale_date = dictionary_record.get('SALEDATE', None)
        p.sale_description = dictionary_record.get('SALEDESC', None)
        p.sale_price = dictionary_record.get('SALEPRICE', None)
        p.school_district_code = dictionary_record.get('SCHOOLCODE', None)
        p.school_district_name = dictionary_record.get('SCHOOLDESC', None)
        p.stories = dictionary_record.get('STORIES', None)
        p.style = dictionary_record.get('STYLE', None)
        p.style = dictionary_record.get('STYLEDESC', None)
        p.tax_code = dictionary_record.get('TAXCODE', None)
        p.tax_description = dictionary_record.get('TAXDESC', None)
        p.tax_subcode = dictionary_record.get('TAXSUBCODE', None)
        p.tax_subcode_description = dictionary_record.get('TAXSUBCODE_DESC', None)
        p.tax_year = dictionary_record.get('TAXYEAR', None)
        p.total_rooms = dictionary_record.get('TOTALROOMS', None)
        p.use_code = dictionary_record.get('USECODE', None)
        p.use_description = dictionary_record.get('USEDESC', None)
        p.year_built = dictionary_record.get('YEARBLT', None)
        return p

class Assessment(Base):
    __tablename__ = 'assessment'

    id = Column(Integer, primary_key=True)
    property_id = Column(Integer, ForeignKey('property.id'))

    # Also from General Information page
    # http://www2.county.allegheny.pa.us/RealEstate/GeneralInfo.aspx?ParcelID=0049F00229000000
    source = Column(String) # Website or database
    year = Column(String) # Year assessment was made
    land_value_dollars = Column(Integer)
    building_value_dollars = Column(Integer)
    total_value_dollars = Column(Integer)


class Building(Base):
    __tablename__ = 'building'

    id = Column(Integer, primary_key=True)
    property_id = Column(Integer, ForeignKey('property.id'))

    # Building Information
    # http://www2.county.allegheny.pa.us/RealEstate/Building.aspx?ParcelID=0049F00229000000
    building_number = Column(Integer)
    use_code = Column(String)
    total_rooms = Column(Integer)
    basement = Column(String)
    style = Column(String)
    bedrooms = Column(Integer)
    grade = Column(String)
    stories = Column(Integer)
    full_baths = Column(Integer)
    condition = Column(String)
    year_built = Column(Integer)
    half_baths = Column(Integer)
    fireplaces = Column(Integer)
    exterior_finish = Column(String)
    heating_cooling = Column(String)
    basement_garage = Column(String)
    roof_type = Column(String)
    living_area_sq_ft = Column(Integer)


class TaxBill(Base):
    __tablename__ = 'tax_bill'

    id = Column(Integer, primary_key=True)
    property_id = Column(Integer, ForeignKey('property.id'))

    # Tax Info page
    # http://www2.county.allegheny.pa.us/RealEstate/Tax.aspx?ParcelID=0049F00229000000
    tax_bill_mailing_address = Column(String)
    year = Column(Integer)
    paid = Column(String)
    status = Column(String)
    tax_amount_cents = Column(Integer)
    penalty_cents = Column(Integer)
    interest_cents = Column(Integer)
    total_cents = Column(Integer)
    date_paid = Column(String) # "YYYY-MM-DD"


class Sale(Base):
    __tablename__ = 'sale'

    id = Column(Integer, primary_key=True)
    property_id = Column(Integer, ForeignKey('property.id'))

    # Owner History page
    # http://www2.county.allegheny.pa.us/RealEstate/Sales.aspx?ParcelID=0049F00229000000
    source = Column(String)
    owner_name = Column(String)
    sale_date = Column(String) # "YYYY-MM-DD"
    sale_price_dollars = Column(Integer)


class Deed(Base):
    __tablename__ = 'deed'

    id = Column(Integer, primary_key=True)
    property_id = Column(Integer, ForeignKey('property.id'))

    # deeds search
    # https://pa_allegheny.uslandrecords.com/palr/controller?commandflag=getDetails&optflag=DetailsCommand&county=pa003&userid=null&userCategory=7&nameid=2878036&ptrno=13250813&partytype=&name=KROOP%2B%2BBENJAMIN%2BC&officeid=60&fromdate=01/01/1986&todate=04/24/2018&lastname=Kroop
    source = Column(String)
    number = Column(Integer)
    file_date = Column(String) # "YYYY-MM-DD"
    type_description = Column(String)
    num_pages = Column(Integer)
    book = Column(String)
    volume = Column(Integer)
    page = Column(Integer)
    consideration_dollars = Column(Integer)
    mailback_details = Column(String)

    grantor_name = Column(String) # semicolon separated, if multiple.
    grantee_name = Column(String) # semicolon separated, if multiple.

    document_status = Column(String)


class Mortgage(Base):
    __tablename__ = 'mortgage'

    id = Column(Integer, primary_key=True)
    property_id = Column(Integer, ForeignKey('property.id'))

    # deeds search
    # https://pa_allegheny.uslandrecords.com/palr/controller?commandflag=getDetails&optflag=DetailsCommand&county=pa003&userid=null&userCategory=7&nameid=2878036&ptrno=13270510&partytype=&name=KROOP%2B%2BBENJAMIN%2BC&officeid=61&fromdate=01/01/1986&todate=04/24/2018&lastname=Kroop
    number = Column(Integer)
    town = Column(String)
    file_date = Column(String) # "YYYY-MM-DD"
    type_description = Column(String)
    instrument_date = Column(String)
    num_pages = Column(String)
    book = Column(String)
    volume = Column(Integer)
    page = Column(Integer)
    consideration_dollars = Column(Integer)
    document_status = Column(String)
    mailback_details = Column(String)
    mortgagor = Column(String) # semicolon separated, if multiple.
    mortgagee = Column(String) # semicolon separated, if multiple.
