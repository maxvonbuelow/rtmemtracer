Ray Tracing Memory Trace Extractor
======

This is the implementation of the memory trace extraction tool for ray tracers. It can be used to visualized the memory and cache behavior of them.

Demo
------
A demo of our visualization tool that uses memory traces from this tool can be found [here](https://riemann.dev/gpu-blame/).

Build instructions
------
First, make sure that you have the [NVBit](https://github.com/NVlabs/NVBit) binaries in nvbit/core. We used version 1.5.5 for our experiments.
```
mkdir build && cd $_
cmake ..
make
```

License & Reference
------
Most parts of our program are licensed under the GPLv3 license included as LICENSE.md file.

If you decide to use our code or code based on this project in your application, please make sure to cite our EGPGV paper:

```
@inproceedings{10.2312/pgv.20221061,
	booktitle = {Eurographics Symposium on Parallel Graphics and Visualization},
	editor = {Bujack, Roxana and Tierny, Julien and Sadlo, Filip},
	title = {{Profiling and Visualizing GPU Memory Access and Cache Behavior of Ray Tracers}},
	author = {Buelow, Max von and Riemann, Kai and Guthe, Stefan and Fellner, Dieter W.},
	year = {2022},
	month = {jun},
	publisher = {The Eurographics Association},
	issn = {1727-348X},
	isbn = {978-3-03868-175-5},
	doi = {10.2312/pgv.20221061},
	location = {Rome, Italy}
}
```


Contact
------
For any trouble with building, using or extending this software, please use the project's integrated issue tracker. We'll be happy to help you there or discuss feature requests.

For requests not matching the above, please contact the maintainer at max.von.buelow(at)gris.tu-darmstadt.de.
