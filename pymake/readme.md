# Deploy

```bash
# After incrementing the version number
rm dist/*
python setup.py sdist
twine upload dist/*
```

# TODO
[ ] Graph required error
[ ] Call cmake from pymake instead of the other way around, for better errors
[ ] Always include backward.cpp in every executable (I guess..!)
[ ] Figure out how to make the backwards stacktraces beautiful, as they were in the old days