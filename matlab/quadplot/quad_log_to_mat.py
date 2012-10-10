#!/usr/bin/env python

#This program converts the .csv log files from the asc tech windows
#software (v1.56 - see quadrotor project on robots) to a usable .mat file.
#
#This requires scipy, which is not included in perls.  However, this
#script is just an temporary tool until we get the quadroter using LCM.

from subprocess import PIPE, Popen, STDOUT
import sys, os, re
import scipy.io
import numpy

NUM_DATA_LABELS = 44
DATE_FORMAT = '%s' #UTC
DATE_COMMAND = 'date' #GNU version of date is required
DEFAULT_DELIMITER = ';'
DATA_FIELD = 'data'
COLHEADERS = 'colheaders'
DATA_RATE = .2

class LogToMatConverter(object):
    """Convert Asc Tech ".csv" to a .mat file
    """
    
    def __init__(self, csvFile, delimiter=DEFAULT_DELIMITER):
        """
        Arguments:
        - `csvFile`: path to csv
        - `delimiter`: default delimiter (Asc Tech uses ';')
        """
        self.csvFile = csvFile
        self.delimiter = delimiter
        self.matData = {}
        self.matData[DATA_FIELD] = []
        self.startTime = None;

        self._readInputFile()

    def _readLabels(self, inFile):
        """
        Read label line, store relevant stuff in self.matData
        """
        #The first line is a bunch of data labels
        firstLine = inFile.readline()
        dataLabelList = self._lineToList(firstLine)
        dataLabelList[0:2] = ['UTC Time'] #Asc Tech time format...

        i = 1
        for _label in dataLabelList:
            _label = re.sub('\s', '', _label) #remove garbage
            _label = re.sub('\.', '', _label) #remove more garbage

            if self.matData.has_key(_label): #handle duplicate keys
                _label = ''.join([_label, '_2'])

            self.matData[_label] = [i] #assign the key to a unique int
            i = i + 1

        #create a matlab cell of data labels
        stringCell = numpy.zeros((NUM_DATA_LABELS - 1), dtype=numpy.object)
        stringCell[:] = dataLabelList
        self.matData[COLHEADERS] = stringCell

    def _readData(self, inFile):
        """
        Read data lines, store relevant stuff in self.matData
        """
        lines = inFile.readlines()
        sample = 0
        for currentLine in lines:

            currentData = self._lineToList(currentLine)

            #set the start time, if it hasn't been set
            if self.startTime is None:
                self.startTime = self._combineDateAndTime(currentData[0], 
                                                          currentData[1])

            currentData[0:2] = [self.startTime + sample*DATA_RATE]
            #convert strings to float
            currentData = [float(thing) for thing in currentData]
            
            self.matData[DATA_FIELD].append(currentData)

            sample = sample + 1

    def _readInputFile(self):
        """
        Read in the contents of the .csv.
        Store data in self.matData, which we will write to a matlab
        file when the user calls self.writeMat
        
        (most of the action happens in self._readLabels and self._readData
        """
        inFile = open(self.csvFile, 'r')
        self._readLabels(inFile)
        self._readData(inFile)

    def _lineToList(self, l):
        """
        Convert the line in the .csv to a usable array of strings"
        """
        l = l.rstrip(self.delimiter + '\n')
        list = l.split(self.delimiter)
        list = list[0:NUM_DATA_LABELS]
        list = [thing.lstrip() for thing in list]

        return list
    
    def _captureOutput(self, args):
        """
        Call system command, and capture the stdout.  Used for getting
        the date with gnu date
        """
        p = Popen(args, stdout=PIPE, stderr=STDOUT)
        output = p.communicate()[0]
        return output

    def _combineDateAndTime(self, date, time):
        """"
        Convert the wonky date and time to seconds from epoch
        """
        splitDate = date.split('.')
        temp = splitDate[0]
        splitDate[0] = splitDate[1]
        splitDate[1] = temp

        dateUsa = '/'.join([item for item in splitDate])
        dateAndTime = ' '.join([dateUsa, time])

        argsToExecute = [DATE_COMMAND, '-d', dateAndTime, '+%s']
        UtcTime = int(self._captureOutput(argsToExecute))
        
        return UtcTime

    def writeMat(self, outFile):
        """
        The user of the class should call this.  
        Create a .mat output file 
        """
        scipy.io.savemat(outFile, self.matData)

def printUsage():

    scriptName = os.path.basename(sys.argv[0])
    print 'Usage:',
    print scriptName + ' <infile.csv> <outfile.mat>'

if __name__ == '__main__':

    if len(sys.argv) != 3:
        printUsage()
        sys.exit(1)

    infile = sys.argv[1]
    outfile = sys.argv[2]

    importer = LogToMatConverter(infile)
    importer.writeMat(outfile)
