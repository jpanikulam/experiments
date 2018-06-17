import numpy as np

from bokeh.io import show
from bokeh.models import (
    ColumnDataSource,
    HoverTool,
    LogColorMapper,
    BoxSelectTool
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
    ftrs = geojson['features']
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
    TOOLS = "pan,wheel_zoom,reset,hover,box_select"

    p = figure(
        title='Allegheny County Parcels',
        tools=TOOLS,
        x_axis_location=None,
        y_axis_location=None,
        # output_backend="webgl"
    )

    p.patches(
        'x', 'y', source=source,
        # fill_color={'field': 'rate', 'transform': color_mapper},
        # fill_alpha=0.7, line_color="white", line_width=0.5
    )

    hover = p.select_one(HoverTool)
    hover.point_policy = "follow_mouse"
    hover.tooltips = [
        ("Name", "@name"),
        ("(Long, Lat)", "($x, $y)"),
    ]

    box = p.select_one(BoxSelectTool)
    box.point_policy = "follow_mouse"

    show(p)


def main():
    file = 'reduced.json'
    geojson = gather_geojson(file)
    generate_parcel_images(geojson)


if __name__ == '__main__':
    main()
