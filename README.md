# Python-based-application-to-extract-u-blox-positioning-data
The objective of this project was to extract the positioning data measured by an u-blox 8T receiver using the UBX protocol (open source pyubx2 library), in particular the recovery of pseudoranges, carrier phase, and ephemeris information. Based on this information, the estimated position of the user in both the Cartesian Coordinate System and Geographical Coordinate System is calculated using customized algorithms introduced in the prior work (https://github.com/AreebTariq/Python-based-application-to-extract-u-blox-positioning-data/blob/main/Ahmed_Tsoroev_PFE_Rapport.pdf). A python-based software application was created which can extract the required information, process them to meaningful form, estimate position, and convert it into autogenerated Excel spreadsheets, all in one click operation.

# Note
Extract the zipped files 'pyubx2' and '_pycache_' in the same folder as all other files. Follow the 'How_to_use.pdf' file for all the detailed operation. 
