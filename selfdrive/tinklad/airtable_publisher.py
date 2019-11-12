#!/usr/bin/env python3
# Created by Raf 5/2019

import asyncio

AIRTABLE_API_KEY = 'keyvqdsl2VIIr9Q2A'
AIRTABLE_BASE_ID = 'appht7GB4aJS2A0LD'

USERS_TABLE = 'Users'
EVENTS_TABLE = 'Events'

LOG_PREFIX = "tinklad.airtable_publisher: "

class AirtableUsersKeys():
    openPilotId = "openPilotId"
    timestamp = "timestamp"
    userHandle = "userHandle"
    gitRemote = "gitRemote"
    gitBranch = "gitBranch"

class AirtableEventKeys():
    openPilotId = "openPilotId"
    timestamp = "timestamp"
    source = "source"
    category = "category"
    name = "name"
    value = "value"

# This needs to match tinkla.capnp
class TinklaEventValueTypes():
    boolValue = 'boolValue'
    textValue = 'textValue'
    intValue = 'intValue'
    floatValue = 'floatValue'

class Publisher():
    openPilotId = None
    latest_info_dict = None # current info published
    userRecordId = None

    async def send_info(self, info, isData= False):
        data_dict = None
        if isData:
            data_dict = info
        else:
            data_dict = self.__generate_airtable_user_info_dict(info)

        # Early return if no changes
        if self.latest_info_dict is not None:
            print(LOG_PREFIX + "latest_info. data=%s" % (self.latest_info_dict)) 
            if data_dict == self.latest_info_dict:
                print(LOG_PREFIX + "send_info no update necessary*")
                return

        print(LOG_PREFIX + "Sending info. data=%s" % (data_dict))
        if self.userRecordId is not None:
            await self.__update_user(data_dict)

        if (info.openPilotId is not None) and info.openPilotId != '':
            self.openPilotId = info.openPilotId

        response = await self.at.get(USERS_TABLE, limit=1, filter_by_formula=("{openPilotId} = '%s'" % (self.openPilotId)))
        if self.__is_notfound_response(response): # Not found, create:
            print(LOG_PREFIX + "Creating record for openPilotId='%s'" % (info.openPilotId))
            response = await self.at.create(USERS_TABLE, data_dict)
            if self.__is_error_response(response):
                raise Exception(response)
        elif self.__is_error_response(response): #Unsupported error
            print(LOG_PREFIX + "Error retrieving data: '%s'" % (response))
            raise Exception(response)
        else:
            self.userRecordId = response["records"][0]["id"]
            await self.__update_user(data_dict)
        
        self.latest_info_dict = data_dict
        print(LOG_PREFIX + "*send_info competed*")

    async def send_event(self, event):
        if self.openPilotId is None and self.latest_info_dict is not None:
            self.openPilotId = self.latest_info_dict[self.userKeys.openPilotId]

        event_dict = self.__generate_airtable_user_event_dict(event)
        print(LOG_PREFIX + "Sending event. data=%s" % (event_dict))
        response = await self.at.create(EVENTS_TABLE, event_dict)
        if self.__is_error_response(response):
            print(LOG_PREFIX + "Error sending airtable event. %s" % (response))
            raise Exception(response)
        print(LOG_PREFIX + "*send_event competed*")


    def __generate_airtable_user_info_dict(self, info):
        dictionary = info.to_dict()
        dictionary.pop(self.userKeys.timestamp, None)
        return dictionary

    def __generate_airtable_user_event_dict(self, event):
        value = event.value.which()
        if value == self.eventValueTypes.boolValue:
            value = event.value.boolValue
        elif value == self.eventValueTypes.textValue:
            value = event.value.textValue
        elif value == self.eventValueTypes.intValue:
            value = event.value.intValue
        elif value == self.eventValueTypes.floatValue:
            value = event.value.floatValue
        openPilotId = event.openPilotId if (event.openPilotId is not None) else (self.openPilotId if (self.openPilotId is not None) else "")
        dictionary = event.to_dict()
        dictionary[self.eventKeys.value] = value
        dictionary[self.eventKeys.openPilotId] = openPilotId
        # dictionary.pop("timestamp", None)
        return dictionary

    async def __update_user(self, data):
        print(LOG_PREFIX + "Updating userRecordId='%s'" % (self.userRecordId))
        response = await self.at.update(USERS_TABLE, self.userRecordId, data)
        if self.__is_error_response(response):
            raise Exception(response)

    def __is_notfound_response(self, response):
        try:
            return response["error"] is not None and response["error"]["code"] == 422
        except: # pylint: disable=bare-except 
            count = response["records"].__len__()
            return count == 0

    def __is_error_response(self, response):
        try:
            return response["error"] is not None
        except: # pylint: disable=bare-except 
            return False

    def __init__(self):
        self.eventValueTypes = TinklaEventValueTypes()
        self.userKeys = AirtableUsersKeys()
        self.eventKeys = AirtableEventKeys()
        self.at = Airtable(AIRTABLE_BASE_ID, AIRTABLE_API_KEY)

################################################################
# airtable.py - https://github.com/josephbestjames/airtable.py #
################################################################

import json
import posixpath 
import requests
import six
from collections import OrderedDict

API_URL = 'https://api.airtable.com/v%s/'
API_VERSION = '0'


class IsNotInteger(Exception):
    pass


class IsNotString(Exception):
    pass


def check_integer(n):
    if not n:
        return False
    elif not isinstance(n, six.integer_types):
        raise IsNotInteger('Expected an integer')
    else:
        return True


def check_string(s):
    if not s:
        return False
    elif not isinstance(s, six.string_types):
        raise IsNotString('Expected a string')
    else:
        return True


def create_payload(data):
    return {'fields': data}


class Airtable():
    def __init__(self, base_id, api_key, dict_class=OrderedDict):
        """Create a client to connect to an Airtable Base.

        Args:
            - base_id: The ID of the base, e.g. "appA0CDAE34F"
            - api_key: The API secret key, e.g. "keyBAAE123C"
            - dict_class: the class to use to build dictionaries for returning
                  fields. By default the fields are kept in the order they were
                  returned by the API using an OrderedDict, but you can switch
                  to a simple dict if you prefer.
        """
        self.airtable_url = API_URL % API_VERSION
        self.base_url = posixpath.join(self.airtable_url, base_id)
        self.headers = {'Authorization': 'Bearer %s' % api_key}
        self._dict_class = dict_class

    def __perform_request(self, method, url, params, data, headers):
        return requests.request(
            method,
            url,
            params=params,
            data=data,
            headers=headers
        )

    async def __request(self, method, url, params=None, payload=None):
        if method in ['POST', 'PUT', 'PATCH']:
            self.headers.update({'Content-type': 'application/json'})
        loop = asyncio.get_event_loop()
        r = await loop.run_in_executor(
            None, 
            self.__perform_request,
            method,
            posixpath.join(self.base_url, url),
            params,
            payload,
            self.headers
        )
        if r.status_code == requests.codes.ok: # pylint: disable=no-member
            return r.json(object_pairs_hook=self._dict_class)
        else:
            try:
                message = None
                r.raise_for_status()
            except requests.exceptions.HTTPError as e:
                message = str(e)
            return {
                'error': dict(code=r.status_code, message=message)
            }

    def get( # pylint: disable=dangerous-default-value
            self, table_name, record_id=None, limit=0, offset=None,
            filter_by_formula=None, view=None, max_records=0, fields=[]):
        params = {}
        if check_string(record_id):
            url = posixpath.join(table_name, record_id)
        else:
            url = table_name
            if limit and check_integer(limit):
                params.update({'pageSize': limit})
            if offset and check_string(offset):
                params.update({'offset': offset})
            if filter_by_formula is not None:
                params.update({'filterByFormula': filter_by_formula})
            if view is not None:
                params.update({'view': view})
            if max_records and check_integer(max_records):
                params.update({'maxRecords': max_records})
            if fields and type(fields) is list: # pylint: disable=unidiomatic-typecheck
                for field in fields: check_string(field)
                params.update({'fields': fields})

        return self.__request('GET', url, params)

    def iterate( # pylint: disable=dangerous-default-value
            self, table_name, batch_size=0, filter_by_formula=None, 
            view=None, max_records=0, fields=[]):
        """Iterate over all records of a table.

        Args:
            table_name: the name of the table to list.
            batch_size: the number of records to fetch per request. The default
                (0) is using the default of the API which is (as of 2016-09)
                100. Note that the API does not allow more than that (but
                allow for less).
            filter_by_formula: a formula used to filter records. The formula
                will be evaluated for each record, and if the result is not 0,
                false, "", NaN, [], or #Error! the record will be included in
                the response. If combined with view, only records in that view
                which satisfy the formula will be returned.
            view: the name or ID of a view in the table. If set, only the
                records in that view will be returned. The records will be
                sorted according to the order of the view.
        Yields:
            A dict for each record containing at least three fields: "id",
            "createdTime" and "fields".
        """
        offset = None
        while True:
            response = self.get(
                table_name, limit=batch_size, offset=offset, max_records=max_records, 
                fields=fields, filter_by_formula=filter_by_formula, view=view)
            for record in response.pop('records'):
                yield record
            if 'offset' in response:
                offset = response['offset'].encode('ascii','ignore')
            else:
                break

    async def create(self, table_name, data): # pylint: disable=inconsistent-return-statements
        if check_string(table_name):
            payload = create_payload(data)
            return await self.__request('POST', table_name,
                                  payload=json.dumps(payload))

    async def update(self, table_name, record_id, data): # pylint: disable=inconsistent-return-statements
        if check_string(table_name) and check_string(record_id):
            url = posixpath.join(table_name, record_id)
            payload = create_payload(data)
            return await self.__request('PATCH', url,
                                  payload=json.dumps(payload))

    async def update_all(self, table_name, record_id, data): # pylint: disable=inconsistent-return-statements
        if check_string(table_name) and check_string(record_id):
            url = posixpath.join(table_name, record_id)
            payload = create_payload(data)
            return await self.__request('PUT', url,
                                  payload=json.dumps(payload))

    async def delete(self, table_name, record_id): # pylint: disable=inconsistent-return-statements
        if check_string(table_name) and check_string(record_id):
            url = posixpath.join(table_name, record_id)
            return await self.__request('DELETE', url)
