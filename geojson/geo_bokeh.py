import numpy as np

from bokeh.io import show
from bokeh.models import (
    ColumnDataSource,
    HoverTool,
    LogColorMapper
)
from bokeh.palettes import Viridis6 as palette
from bokeh.plotting import figure
import json

# import bokeh
# bokeh.sampledata.download()
# from bokeh.sampledata.us_counties import data as counties
# from bokeh.sampledata.unemployment import data as unemployment


def gather_geojson(file):
    with open(file) as f:
        txt = f.read()
        return json.loads(txt)


def get_points(parcel):
    # assert parcel['geometry']['type'] == u'Polygon'
    return np.asarray(parcel['geometry']['coordinates'][0])


def generate_parcel_images(geojson):
    ftrs = geojson['features'][:1000]
    # parcel_names = [f['properties']['PIN'] for f in ftrs]
    parcel_names = []
    parcel_xs = []
    parcel_ys = []
    for parcel in ftrs:
        pts = get_points(parcel)
        if pts.ndim != 2:
            continue
        parcel_names.append(parcel['properties']['PIN'])
        # print pts.shape
        parcel_xs.append(pts[:, 0])
        parcel_ys.append(pts[:, 1])

    color_mapper = LogColorMapper(palette=palette)
    source = ColumnDataSource(data={
        'x': parcel_xs,
        'y': parcel_ys,
        'name': parcel_names
    })
    TOOLS = "pan,wheel_zoom,reset,hover,save"

    p = figure(
        title='Allegheny County Parcels',
        tools=TOOLS,
        x_axis_location=None,
        y_axis_location=None,
        output_backend="webgl"
    )

    p.patches('x', 'y', source=source)
    hover = p.select_one(HoverTool)
    hover.point_policy = "follow_mouse"
    hover.tooltips = [
        ("Name", "@name"),
        ("(Long, Lat)", "($x, $y)"),
    ]

    show(p)


def main():
    file = 'reduced.json'
    geojson = gather_geojson(file)
    generate_parcel_images(geojson)


if __name__ == '__main__':
    main()
