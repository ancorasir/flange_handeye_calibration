//
// Created by controller on 1/12/18.
//

#ifndef PROJECT_PHOXIEXCEPTION_H
#define PROJECT_PHOXIEXCEPTION_H

#include <exception>
class PhoXiInterfaceException : public std::exception {
public:
    PhoXiInterfaceException(std::string message) : message(message){
    }
    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};

class PhoXiControlNotRunning : public PhoXiInterfaceException {
public:
    PhoXiControlNotRunning(std::string message) : PhoXiInterfaceException(message){
    }
};

class PhoXiScannerNotFound : public PhoXiInterfaceException {
public:
    PhoXiScannerNotFound(std::string message) : PhoXiInterfaceException(message){
    }
};
class UnableToConnect : public PhoXiInterfaceException {
public:
    UnableToConnect(std::string message) : PhoXiInterfaceException(message){
    }
};

class UnableToStartAcquisition : public PhoXiInterfaceException {
public:
    UnableToStartAcquisition(std::string message) : PhoXiInterfaceException(message){
    }
};

class UnableToStopAcquisition : public PhoXiInterfaceException {
public:
    UnableToStopAcquisition(std::string message) : PhoXiInterfaceException(message){
    }
};

class CorruptedFrame : public PhoXiInterfaceException {
public:
    CorruptedFrame(std::string message) : PhoXiInterfaceException(message){
    }
};

class  PhoXiScannerNotConnected : public PhoXiInterfaceException {
public:
    PhoXiScannerNotConnected(std::string message) : PhoXiInterfaceException(message){
    }
};

class  CoordinationSpaceNotSupported : public PhoXiInterfaceException {
    public:
    CoordinationSpaceNotSupported(std::string message) : PhoXiInterfaceException(message){
    }
};

class  InvalidTransformationMatrix : public PhoXiInterfaceException {
public:
    InvalidTransformationMatrix(std::string message) : PhoXiInterfaceException(message){
    }
};

class  InvalidTriggerMode : public PhoXiInterfaceException {
public:
    InvalidTriggerMode(std::string message) : PhoXiInterfaceException(message){
    }
};


#endif //PROJECT_PHOXIEXCEPTION_H
