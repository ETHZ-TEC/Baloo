# Baloo

Baloo is a design framework for network stacks based on Synchronous Transmissions (i.e., using a flooding protocol like [Glossy](https://sourceforge.net/p/contikiprojects/code/HEAD/tree/ethz.ch/glossy/) as underlying communication primitive). Baloo is flexible enough to implement a wide variety of network layer protocols, while introducing only limited memory and energy overhead. 

Using Baloo, one can relatively easily re-implement network layer protocols like the [Low-power Wireless Bus](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/LWBSenSys12.pdf), [Crystal](http://disi.unitn.it/~picco/papers/ipsn18.pdf), and [Sleeping Beauty](http://www.chayansarkar.com/papers/chayan_mass16.pdf).

The general concept of Baloo and its working principles have been presented in the following paper. 
> *Synchronous Transmissions Made Easy: Design Your Network Stack with Baloo*  
Romain Jacob, Jonas Bächli, Reto Da Forno, Lothar Thiele   
Proceedings of the 2019 International Conference on Embedded Wireless Systems and Networks (EWSN) 2019.  
[Direct Link](https://www.research-collection.ethz.ch/handle/20.500.11850/324254)

Unless explicitly stated otherwise, all Baloo sources are distributed under the terms of the [3-clause BSD license](license). This license gives everyone the right to use and distribute the code, either in binary or source code format, as long as the copyright license is retained in the source code.

## Online presence
* GitHub repository   [github.com/ETHZ-TEC/Baloo](https://github.com/ETHZ-TEC/Baloo)
* Project webpage [www.romainjacob.net/baloo/](http://www.romainjacob.net/research/baloo/)

## Disclaimer 
Although we tested the code extensively, Baloo is a research prototype that likely contain bugs. We take no responsibility for and give no warranties in respect of using the code.

## Documentation

The [project Wiki](https://github.com/ETHZ-TEC/Baloo/wiki) documents the Baloo mechanics and features, how they are implemented and how to use them in your designs. 
In addition, the Baloo source files contain detailed Doxygen comments. The Doxygen documentation can be generated using the following commands
```
cd /tools/doxygen/
make html
```
This generates the complete documentation for Contiki-NG. Documentation of Baloo files can be directly accessed  `/tools/doxygen/html/group__gmw.html`

This repository contains an implementation of Baloo using the [Contiki-NG](http://contiki-ng.org/) operating system. Some minor modifications to the original OS code where made. The list of modified files (and rational for changes, whenever appropriate) is available [here](Contiki-change-files-log).
