/*
 * NoStateAvailableException.h
 *
 *  Created on: Jan 29, 2012
 *      Author: maxson
 */

#ifndef NOSTATEAVAILABLEEXCEPTION_H_
#define NOSTATEAVAILABLEEXCEPTION_H_

class NoStateAvailableException {
private:
	int index;
public:
	NoStateAvailableException(int index) : index(index) {}
	int getIndex(){return index;}
};

#endif /* NOSTATEAVAILABLEEXCEPTION_H_ */
