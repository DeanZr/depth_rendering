//----------------------------------------------------------------------------
//
//  Copyright (C) 2022, Northern Digital Inc. All rights reserved.
//
//  All Northern Digital Inc. ("NDI") Media and/or Sample Code and/or Sample Code
//  Documentation (collectively referred to as "Sample Code") is licensed and provided "as
//  is" without warranty of any kind. The licensee, by use of the Sample Code, warrants to
//  NDI that the Sample Code is fit for the use and purpose for which the licensee intends to
//  use the Sample Code. NDI makes no warranties, express or implied, that the functions
//  contained in the Sample Code will meet the licensee's requirements or that the operation
//  of the programs contained therein will be error free. This warranty as expressed herein is
//  exclusive and NDI expressly disclaims any and all express and/or implied, in fact or in
//  law, warranties, representations, and conditions of every kind pertaining in any way to
//  the Sample Code licensed and provided by NDI hereunder, including without limitation,
//  each warranty and/or condition of quality, merchantability, description, operation,
//  adequacy, suitability, fitness for particular purpose, title, interference with use or
//  enjoyment, and/or non infringement, whether express or implied by statute, common law,
//  usage of trade, course of dealing, custom, or otherwise. No NDI dealer, distributor, agent
//  or employee is authorized to make any modification or addition to this warranty.
//  In no event shall NDI nor any of its employees be liable for any direct, indirect,
//  incidental, special, exemplary, or consequential damages, sundry damages or any
//  damages whatsoever, including, but not limited to, procurement of substitute goods or
//  services, loss of use, data or profits, or business interruption, however caused. In no
//  event shall NDI's liability to the licensee exceed the amount paid by the licensee for the
//  Sample Code or any NDI products that accompany the Sample Code. The said limitations
//  and exclusions of liability shall apply whether or not any such damages are construed as
//  arising from a breach of a representation, warranty, guarantee, covenant, obligation,
//  condition or fundamental term or on any theory of liability, whether in contract, strict
//  liability, or tort (including negligence or otherwise) arising in any way out of the use of
//  the Sample Code even if advised of the possibility of such damage. In no event shall
//  NDI be liable for any claims, losses, damages, judgments, costs, awards, expenses or
//  liabilities of any kind whatsoever arising directly or indirectly from any injury to person
//  or property, arising from the Sample Code or any use thereof
//
//----------------------------------------------------------------------------
#ifndef FILE_CONNECTION_HPP
#define FILE_CONNECTION_HPP

#include "Connection.h"

/**
 * Implements a file reader as an extension of the Connection class, to be used for reading from a file instead of a port
 */
class FileConnection : public Connection
{
public:
	/**
	 * @brief Constructs a FileConnection
	 */
	FileConnection();
	~FileConnection();

	/**
	 * If the input file stream is open
	 */
	bool isConnected() const;

	/**
	 * Opens an input file stream to the specified file
	 */
	bool connect( const char* fileName );

	/**
	 * Closes the input file stream
	 */
	void disconnect();

	/**
	 * Reads the specified number of characters from the input file stream
	 */
	int read( char* buffer, int length ) const;

	/**
	 * Reads the specified number of bytes from the input file stream
	 */
	int read( byte_t* buffer, int length ) const;

	/**
	 * Not applicable
	 */
	int write( const char* buffer, int length ) const { return 0; }

	/**
	 * Not applicable
	 */
	int write( byte_t* buffer, int length ) const { return 0; }

	/**
	 * Returns the file name
	 */
	char* connectionName();

	/**
	 * The number of bytes in the file between the current file pointer and the end of the file
	 */
	int getBytesRemaining() const;

private:
	/** input file stream */
	std::ifstream* input;

	/** name of the input file */
	char* filename;

	/** size of the file in bytes */
	int fileSize;
};

#endif // FILE_CONNECTION_HPP