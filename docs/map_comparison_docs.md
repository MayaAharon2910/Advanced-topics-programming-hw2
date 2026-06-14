#MapsComparison CLI Utility

Its API should meet the API as introduced in the reference stub implementation.
No need for any interface, just this class as a standalone.
This utility should also create an additional target (executable), with run instructions:
```
	./maps_comparison <map1> <map2> [comparison_config=<path>]
```

arguments:
`map1` and `map2` will be the .npy file names (with or without path).
Comparison_config is a path to a YAML file with the relevant configuration for executing the comparisons.The optional third argument is `comparison_config=<path>`. The YAML file should contain `comparison_config.original` and `comparison_config.target`, each with `map_res_cm`, `map_offset`, and `map_boundaries`. If not provided, both maps are assumed to share the same offset, boundaries, and resolution.
The program will print to the standard output just the score number
As a floating point number between 0 and 100 - no additional text!
In case of an error: print to standard output the score -1 and to standard error an descriptive error message of your choice.

The actual comparison algorithm is yours, but we will check that:
Two identical maps return 100.
Two very similar maps return a number close to 100, but not 100.
Two very distinct maps return a number close to 0.
Anything in between returns a reasonable result!

# MapsComparison API
