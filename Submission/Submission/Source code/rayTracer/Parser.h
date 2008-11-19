#pragma once
#include "StdAfx.h"
/*
 * Singleton Class
 */
#define MAX_LINE_LEN 500
#define MAX_TOKEN_LEN 100

class Parser
{
	char *fname;
	FILE *fp;
	Image *I;
	char *lineBuff;
	char *tokBuff;

public:
	static Parser* Instance(char *f, Image *I);
	bool initFileHeaders();
	bool getNextCommand();
	Parser::~Parser(void)
	{
		fclose(fp);
		free(lineBuff);
		free(tokBuff);
	}

protected:
	Parser(char *f, Image *I){
		fname = strdup(f);
		lineBuff = (char *)malloc(sizeof(char) * MAX_LINE_LEN);
		tokBuff = (char *)malloc(sizeof(char) * MAX_TOKEN_LEN);
	}
	Parser& operator= (const Parser&);
private:
    static Parser* pinstance;
	bool isEnd(char c);
	bool isWhiteSpace(char c);
	char *skipWhiteSpaces(char *ptr);
	bool isComment(char *ptr);
	char *getNextToken(char *ptr);
};
