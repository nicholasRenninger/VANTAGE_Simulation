import os
import re

__author__ = "Nicholas Renninger"
__copyright__ = "'Copyright' 2019, VANTAGE"
__credits__ = ["Long Song Silver"]
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Nicholas Renninger"
__email__ = "nicholas.renninger@colorado.edu"
__status__ = "Development"


#
# @brief      removes all non-noisy pcd output files
#
# @param      outputDir  The output directory to clean
#
# @return     outputDir now only contains noisy output pcd files
#
def cleanOutputDir(outputDir):

    # black magic and satanic worship used below
    r = re.compile(r'^((?!.*simArchive).).((?!.*noisy).)*(pcd)$')

    # 3AM hacker black magic
    pathsToUnwantedFiles = [os.path.join(dirpath, f)
                            for dirpath, dirnames, files in os.walk(outputDir)
                            for f in list(filter(r.match, files))]

    for file in pathsToUnwantedFiles:
        os.remove(file)


if __name__ == '__main__':

    # Nick LT simulation case drive path
    # outputDir = 'D:\\Google Drive\\Undergrad\\VANTAGE\\13 Simulation\\4_Simulation_Cases'

    # Nick Workstation simulation case drive path
    outputDir = 'F:\\Cloud\\Google Drive\\Undergrad\\VANTAGE\\13 Simulation\\4_Simulation_Cases'

    cleanOutputDir(outputDir)
