#include "StdAfx.h"
#include "Parser.h"

Parser* Parser::pinstance = 0;

Parser* Parser::Instance(char *f, Image *Img)
{
	if(pinstance==0)
	{
		pinstance = new Parser(f, Img);
	}
	return pinstance;
}

bool Parser::isEnd(char c)
{
	return(c == '\0' || c == '\n');
}

bool Parser::isWhiteSpace(char c)
{
	return(c == ' ' || c == '\t');
}

/* 
 * returns pointer to next valid string in input set
 * returns '\0' or '\n' if string termination is encountered 
 */
char *Parser::skipWhiteSpaces(char *ptr)
{
	while(!isEnd(*ptr))
	{
		if(isWhiteSpace(*ptr))
			ptr++;
		else
			break;
	}
	return ptr;
}

bool Parser::isComment(char *ptr)
{
	return(*ptr=='#');
}

char *Parser::getNextToken(char *ptr)
{
	int i = 0;

	while(!isEnd(*ptr) && !isWhiteSpace(*ptr))
		tokBuff[i++]=*ptr++;
	tokBuff[i] = '\0';	/* serves to clear tokBuff too */

	return ptr;
}

bool Parser::initFileHeaders()
{
	assert(fp = fopen(fname, "rt")) ;

	/* set size */
	if(!getNextCommand())
	{
		fprintf(stderr, "Parser:: Empty file.");
		return false;
	}
	assert(C->cmd=CMD_SIZE);

	assert(C->updateCommandArgs());

	return true;
}

bool Parser::getNextCommand()
{
	char *ptr = lineBuff;
read:
	if(fgets(lineBuff, MAX_LINE_LEN, fp))
	{
		printf("\nline: %s", lineBuff);
		//scanf("%d", &i);
		if(isComment(ptr))
			goto read;
		ptr = skipWhiteSpaces(ptr);
		if(*ptr=='\0' || *ptr =='\n')
			goto read;

		/* command */
		ptr = getNextToken(ptr);
		printf("\ttoken: %s", tokBuff);
		//scanf("%d", &i);
		assert(C->setCommand(tokBuff));

		/* args */
		tokBuff[0] = '\0';
		while(!isEnd(*ptr))
		{
			ptr = skipWhiteSpaces(ptr);

			if(isEnd(*ptr))
				break;

			ptr = getNextToken(ptr);
			printf("\ttoken: %s", tokBuff);
		//scanf("%d", &i);

			assert(C->setArg(tokBuff));
		}
		/* no arg found */
		if(tokBuff[0]=='\0')
			assert(C->setArg(tokBuff));

		return true;
	}
	else
	{
		fprintf(stderr, "\nParser:: No more commands left to read, EOF");
		return false;
	}
}
